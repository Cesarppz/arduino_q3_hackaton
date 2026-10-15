"""Modo híbrido: YOLO encuentra el globo, VitTrack lo sigue y YOLO lo re-verifica.

YOLO por el brick (HTTP) tarda mucho por frame; el tracker VitTrack de OpenCV
(object_tracking_vittrack_2023sep.onnx, ~700 KB) tarda unos ms. Flujo:

- Modo YOLO: YOLO en cada frame (como sin tracker).
- Cuando el controlador está en APPROACH con un objetivo recién detectado y
  todavía lejos, se inicia el tracker sobre su caja → modo TRACK.
- Modo TRACK: solo el tracker en cada frame. Cada `verify_every` s se lanza YOLO
  en un hilo sobre ese frame; el control no se detiene mientras tanto. Si YOLO
  no confirma que la caja seguida es un globo del color objetivo (u otro globo
  se mete en el camino), se suelta el tracker.
- Se vuelve a YOLO en el mismo frame si baja el score del tracker, si el globo
  ya está cerca (embestir y abortar se decide con YOLO fresco) o si el estado
  deja de ser APPROACH.
"""
import time
from concurrent.futures import ThreadPoolExecutor

import cv2

from car_core import APPROACH, classify_detections, get_detections, label_to_color


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def _box(d):
    return (d["x1"], d["y1"], d["x2"], d["y2"])


class BalloonTracker:
    """cv2.TrackerVit que devuelve la caja con el formato de `get_detections`."""

    def __init__(self, model_path, min_score=0.4):
        if not hasattr(cv2, "TrackerVit_create"):
            raise RuntimeError(f"OpenCV {cv2.__version__} no tiene TrackerVit (hace falta >= 4.9)")
        self.params = cv2.TrackerVit_Params()
        self.params.net = model_path
        self.min_score = min_score
        self.tracker = cv2.TrackerVit_create(self.params)  # falla aquí si el .onnx no carga
        self.label = None
        self.score = 0.0

    def init(self, frame, det):
        self.tracker = cv2.TrackerVit_create(self.params)  # estado limpio en cada objetivo
        x1, y1 = int(det["x1"]), int(det["y1"])
        w, h = max(int(det["x2"] - x1), 2), max(int(det["y2"] - y1), 2)
        self.tracker.init(frame, (x1, y1, w, h))
        self.label = det["label"]
        self.score = 1.0

    def update(self, frame):
        """Caja seguida o None si el tracker la perdió."""
        ok, (x, y, w, h) = self.tracker.update(frame)
        self.score = float(self.tracker.getTrackingScore())
        if not ok or self.score < self.min_score or w <= 0 or h <= 0:
            return None
        fh, fw = frame.shape[:2]
        x1, y1 = max(0.0, float(x)), max(0.0, float(y))
        x2, y2 = min(float(fw), float(x + w)), min(float(fh), float(y + h))
        return {"label": self.label, "color": label_to_color(self.label), "confidence": self.score,
                "x1": x1, "y1": y1, "x2": x2, "y2": y2, "x": (x1 + x2) / 2, "y": y2,
                "width": (x2 - x1) / fw, "bottom": y2 / fh, "tracked": True}


class HybridPerception:
    """Da las detecciones de cada frame usando YOLO o el tracker (o solo YOLO si tracker=None)."""

    def __init__(self, detector, cfg, tracker=None, log=print):
        self.detector, self.cfg, self.tracker, self.log = detector, cfg, tracker, log
        self.model_w, self.model_h = detector.dim
        self.mode = "YOLO"
        self.pool = ThreadPoolExecutor(max_workers=1) if tracker else None
        self.pending = None        # (future, caja seguida al lanzar, w, h)
        self.last_verify = 0.0
        self.misses = 0
        self.yolo_calls = 0
        self.ms = 0.0              # duración de la última percepción

    def _detect(self, frame):
        self.yolo_calls += 1
        res = self.detector.infer(frame)
        return self._parse(res, frame)

    def _parse(self, res, frame):
        h, w = frame.shape[:2]
        dets = get_detections(res, w, h, self.model_w, self.model_h,
                              self.detector.resize_mode, self.cfg.min_confidence)
        return classify_detections(frame, dets, self.cfg)

    def _drop(self, reason):
        if self.pending:  # el brick no debe recibir dos peticiones a la vez
            self.pending[0].result()
            self.pending = None
        self.mode = "YOLO"
        self.log(f"  tracker → YOLO ({reason})")

    def _poll(self, ctrl):
        """Procesa la verificación YOLO en segundo plano si ya terminó."""
        if not self.pending or not self.pending[0].done():
            return
        fut, box, frame, (w, h) = self.pending
        self.pending = None
        dets = self._parse(fut.result(), frame)
        c = self.cfg
        same = [d for d in dets if iou(_box(d), box) >= c.verify_iou]
        good = [d for d in same if d["color"] == c.target_color]
        if any(d["color"] != c.target_color for d in same) and not good:
            self._drop("YOLO dice que es otro color")
        elif not good:
            self.misses += 1
            if self.misses >= c.verify_misses:
                self._drop(f"YOLO no lo ve {self.misses} veces")
        else:
            self.misses = 0
            obstacles = [d for d in dets if d["color"] != c.target_color]
            if ctrl._blocking(obstacles, good[0], w, h):
                self._drop("globo de otro color en el camino")

    def get(self, frame, ctrl, now=None):
        now = time.monotonic() if now is None else now
        t0 = time.perf_counter()
        if self.mode == "TRACK":
            self._poll(ctrl)
        if self.mode == "TRACK":
            if ctrl.state != APPROACH:
                self._drop(f"estado {ctrl.state}")
            else:
                det = self.tracker.update(frame)
                if det is None:
                    self._drop(f"score {self.tracker.score:.2f}")
                elif ctrl._is_near(det):
                    self._drop("globo cerca")
                else:
                    if not self.pending and now - self.last_verify >= self.cfg.verify_every:
                        h, w = frame.shape[:2]
                        snap = frame.copy()
                        self.pending = (self.pool.submit(self.detector.infer, snap),
                                        _box(det), snap, (w, h))
                        self.yolo_calls += 1
                        self.last_verify = now
                    self.ms = (time.perf_counter() - t0) * 1000
                    return [det]
        dets = self._detect(frame)
        self.ms = (time.perf_counter() - t0) * 1000
        return dets

    def after_step(self, frame, out, ctrl, now=None):
        """Tras `ctrl.step`: arranca el tracker sobre el objetivo recién detectado por YOLO."""
        if not self.tracker or self.mode != "YOLO" or out["state"] != APPROACH:
            return
        t = out["target"]
        fresh = t is not None and any(_box(d) == _box(t) for d in out["targets"])
        if not fresh or ctrl._is_near(t):
            return
        self.tracker.init(frame, t)
        self.mode = "TRACK"
        self.misses = 0
        self.last_verify = time.monotonic() if now is None else now  # YOLO acaba de verlo
        self.log("  YOLO → tracker")

    def status(self, now=None):
        if self.mode != "TRACK":
            return "YOLO"
        now = time.monotonic() if now is None else now
        nxt = max(0.0, self.cfg.verify_every - (now - self.last_verify))
        return f"TRACK score={self.tracker.score:.2f} verif {'...' if self.pending else f'{nxt:.1f}s'}"

    def stop(self):
        if self.pool:
            self.pool.shutdown(wait=True)

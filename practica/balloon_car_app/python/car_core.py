"""Carro revienta-globos: detecciones YOLO → selección de objetivo → control (v, w) → MCU.

El modelo es un YOLO-Pro de Edge Impulse (`.eim`, etiquetas `azul`, `rojo`,
`verde`). A diferencia de FOMO, sus `bounding_boxes` tienen tamaño real, y eso
se usa para la distancia:

- La cámara va muy baja: la parte de arriba del globo suele salir cortada
  (caja con y=0). Por eso el centro vertical de la caja no sirve; se usa el
  **borde inferior** (la base del globo en el suelo): cuanto más abajo, más cerca.
- El **ancho** de la caja también crece al acercarse.

Piezas:
- `get_detections`: cajas del SDK → coordenadas del frame + color por etiqueta.
- `classify_detections`: (opcional) confirma el color con HSV dentro de la caja.
- `BalloonController`: SEARCH → APPROACH → COMMIT → BACKOFF → VERIFY → (SEARCH | DONE).
  Se llama a ~20 Hz; YOLO llega mucho más lento y entre frames `step(None, ...)`.
- `DriveSender` + transportes: envían (v, w) en [-100, 100] al sketch.

Convenciones: v > 0 = adelante; w > 0 = girar a la DERECHA. El sketch mezcla
izquierdo = v + w, derecho = v − w.
"""
# %% [imports]
import math
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from fomo_core import model_to_frame, pick_target

COLORS = ("rojo", "verde", "azul")

# Palabras que identifican el color en la etiqueta del modelo
_LABEL_KEYWORDS = {
    "rojo": ("rojo", "red", "roja"),
    "verde": ("verde", "green"),
    "azul": ("azul", "blue"),
}


# %% [config]
@dataclass
class CarConfig:
    target_color: str = "rojo"
    pops_goal: int = 3

    # --- Detección / color ---
    min_confidence: float = 0.5
    # Confirmar el color de YOLO con HSV dentro de la caja. Apagado por defecto:
    # con el sol de la pista el verde sale casi blanco y HSV lo rechazaría.
    use_hsv: bool = False
    hsv_ranges: dict = field(default_factory=lambda: {
        "rojo": [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (179, 255, 255))],
        "verde": [((35, 50, 50), (85, 255, 255))],
        "azul": [((95, 90, 50), (130, 255, 255))],
    })
    min_color_frac: float = 0.15    # fracción mínima de píxeles del color en la caja

    # --- PID sobre dx_norm ([-1, 1]) → w ---
    kp: float = 88.0
    ki: float = 19.0
    kd: float = 10.0
    i_limit: float = 31.0           # anti-windup (en unidades de w)
    w_max: float = 75.0
    smoothing: float = 0.6          # EMA del punto objetivo (1.0 = sin suavizar)
    hold_frames: int = 2            # frames YOLO que se mantiene el objetivo si se pierde (≈1.4 s)
    confirm_frames: int = 1         # frames viendo el objetivo para salir de SEARCH (1 = ir ya)

    # --- Control a 20 Hz con YOLO lento (≈0.7 s por frame) ---
    w_hold: float = 0.3             # s que se aplica el giro tras cada frame (evita pasarse)
    stale_stop: float = 2.0         # s sin percepción nueva en APPROACH → parar

    # --- Velocidades (unidades -100..100) ---
    v_max: float = 85.0
    v_min: float = 40.0
    align_dx: float = 0.5           # |dx| mayor → girar en sitio (v = 0)
    v_commit: float = 100.0         # embestida a tope
    v_back: float = 63.0
    w_search: float = 60.0
    v_wander: float = 50.0

    # --- Cercanía (borde inferior de la caja y ancho, en fracción del frame) ---
    near_bottom: float = 0.50       # dataset: a punto de tocar ≈ 0.6–0.7; antes por la latencia
    near_width: float = 0.38        # dataset: lejos ≈ 0.3, muy cerca ≈ 0.8–0.9
    commit_dx: float = 0.25         # |dx| máximo para embestir

    # --- Tiempos (s) ---
    t_commit: float = 1.2
    t_back: float = 0.5
    verify_settle: float = 0.15     # VERIFY usa un frame tomado al menos esto después de parar
    verify_max: float = 2.5         # sin frame en este tiempo → se decide sin verlo
    search_on: float = 0.30         # pulso de giro en SEARCH (más largo = más ángulo por pulso)
    search_settle: float = 0.10     # tras el pulso, mirar con un frame tomado ya parado
    search_wait_max: float = 2.5    # máximo esperando ese frame
    search_pulses: int = 16         # pulsos sin ver nada → WANDER
    t_wander: float = 1.0
    max_retries: int = 2

    # --- Evitar globos de otro color ---
    corridor: float = 0.20          # medio ancho del carro en la imagen (fracción de media imagen)
    obstacle_bottom: float = 0.30   # solo cuentan los obstáculos con la base más abajo de esto
    k_repel: float = 56.0      # +25 % (antes 45)

    # --- Modo híbrido YOLO + VitTrack (tracker.py) ---
    use_tracker: bool = True        # seguir el objetivo con VitTrack entre inferencias YOLO
    verify_every: float = 3.0       # s entre verificaciones YOLO mientras se sigue (2–4)
    track_min_score: float = 0.4    # score del tracker por debajo → se suelta
    verify_iou: float = 0.3         # IoU mínimo entre la caja seguida y la de YOLO
    verify_misses: int = 2          # verificaciones seguidas sin ver el globo → se suelta


# %% [detections]
def label_to_color(label):
    """Color de una etiqueta ('rojo', 'globo_red', ...) o None si no lo indica."""
    s = label.lower()
    for color, words in _LABEL_KEYWORDS.items():
        if any(w in s for w in words):
            return color
    return None


def get_detections(res, frame_w, frame_h, model_w, model_h, mode, min_confidence=0.5):
    """Cajas YOLO del SDK en coordenadas del frame.

    Cada detección: label, color, confidence, x1, y1, x2, y2,
    x (centro horizontal), y (borde inferior = base del globo),
    width (ancho / W) y bottom (y2 / H).
    """
    dets = []
    for bb in res.get("result", {}).get("bounding_boxes", []):
        if bb["value"] < min_confidence:
            continue
        x1, y1 = model_to_frame(bb["x"], bb["y"], frame_w, frame_h, model_w, model_h, mode)
        x2, y2 = model_to_frame(bb["x"] + bb["width"], bb["y"] + bb["height"],
                                frame_w, frame_h, model_w, model_h, mode)
        x1, x2 = max(0.0, x1), min(float(frame_w), x2)
        y1, y2 = max(0.0, y1), min(float(frame_h), y2)
        dets.append({"label": bb["label"], "color": label_to_color(bb["label"]),
                     "confidence": bb["value"], "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                     "x": (x1 + x2) / 2, "y": y2,
                     "width": (x2 - x1) / frame_w, "bottom": y2 / frame_h})
    return dets


def color_mask(hsv, color, cfg: CarConfig):
    mask = None
    for lo, hi in cfg.hsv_ranges[color]:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def verify_color(frame_bgr, det, cfg: CarConfig):
    """Color HSV dominante en la zona central de la caja (sin la base negra) o None."""
    x1, x2 = det["x1"], det["x2"]
    y1, y2 = det["y1"], det["y2"]
    bw, bh = x2 - x1, y2 - y1
    xa, xb = int(x1 + 0.25 * bw), int(x2 - 0.25 * bw)
    ya, yb = int(y1 + 0.15 * bh), int(y2 - 0.30 * bh)
    if xb - xa < 4 or yb - ya < 4:
        return None
    hsv = cv2.cvtColor(frame_bgr[ya:yb, xa:xb], cv2.COLOR_BGR2HSV)
    fracs = {c: cv2.countNonZero(color_mask(hsv, c, cfg)) / hsv[..., 0].size for c in COLORS}
    color = max(fracs, key=fracs.get)
    return color if fracs[color] >= cfg.min_color_frac else None


def classify_detections(frame_bgr, dets, cfg: CarConfig):
    """Con `use_hsv`, un globo cuyo color YOLO no confirma HSV queda con color None
    y se trata como obstáculo (más vale esquivar un globo dudoso que reventar uno equivocado)."""
    if not cfg.use_hsv or frame_bgr is None:
        return dets
    out = []
    for d in dets:
        hsv_color = verify_color(frame_bgr, d, cfg)
        out.append({**d, "color": d["color"] if hsv_color == d["color"] else None})
    return out


# %% [pid]
class PID:
    def __init__(self, kp, ki, kd, i_limit, out_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.out_limit = i_limit, out_limit
        self.reset()

    def reset(self):
        self.i = 0.0
        self.prev = None

    def update(self, err, dt):
        dt = max(dt, 1e-3)
        self.i = float(np.clip(self.i + self.ki * err * dt, -self.i_limit, self.i_limit))
        d = 0.0 if self.prev is None else (err - self.prev) / dt
        self.prev = err
        return float(np.clip(self.kp * err + self.i + self.kd * d, -self.out_limit, self.out_limit))


class PointFilter:
    """EMA del punto objetivo y tolerancia a pérdidas breves (como CentroidFilter)."""

    def __init__(self, smoothing, hold_frames):
        self.alpha, self.hold_frames = smoothing, hold_frames
        self.pos, self.det, self.missed = None, None, 0

    def update(self, det):
        """Devuelve (detección filtrada, held) o (None, False) si se perdió."""
        if det is not None:
            if self.pos is None or self.missed > 0:
                self.pos = (det["x"], det["y"])
            else:
                a = self.alpha
                self.pos = (a * det["x"] + (1 - a) * self.pos[0],
                            a * det["y"] + (1 - a) * self.pos[1])
            self.det, self.missed = det, 0
            return {**det, "x": self.pos[0], "y": self.pos[1]}, False
        if self.pos is not None and self.missed < self.hold_frames:
            self.missed += 1
            return {**self.det, "x": self.pos[0], "y": self.pos[1]}, True
        self.pos = self.det = None
        return None, False


# %% [controller]
SEARCH, WANDER, APPROACH, COMMIT, BACKOFF, VERIFY, DONE = (
    "SEARCH", "WANDER", "APPROACH", "COMMIT", "BACKOFF", "VERIFY", "DONE")


def _norm_x(x, w):
    return (x - w / 2) / (w / 2)


class BalloonController:
    """Decide (v, w) a partir de las detecciones de cada frame."""

    def __init__(self, cfg: CarConfig):
        self.cfg = cfg
        self.pid = PID(cfg.kp, cfg.ki, cfg.kd, cfg.i_limit, cfg.w_max)
        self.filter = PointFilter(cfg.smoothing, cfg.hold_frames)
        self.popped = 0
        self.retries = 0
        self.search_dir = 1
        self.confirm = 0
        self.last_near = False
        self.aborted = False        # la última embestida se abortó por un obstáculo
        self.last_t = None          # hora del último frame nuevo (dt del PID)
        self.last_fresh = time.monotonic()
        self.seen = ([], [])        # (targets, obstacles) del último frame nuevo
        self._enter(SEARCH, time.monotonic())

    def _enter(self, state, now):
        self.state = state
        self.t_state = now
        self.pid.reset()
        self.confirm = 0
        self.cmd, self.cmd_t = (0.0, 0.0), now   # último comando calculado con un frame
        self.target = None
        if state in (SEARCH, WANDER):
            self.filter = PointFilter(self.cfg.smoothing, self.cfg.hold_frames)
        if state == SEARCH:
            self.pulses = 0
            self.pulse_end = now    # primero mirar, luego girar

    def _is_near(self, d):
        return d["bottom"] >= self.cfg.near_bottom or d["width"] >= self.cfg.near_width

    def _blocking(self, obstacles, target, w, h):
        """Obstáculos más cercanos que el objetivo cuya caja invade el camino hacia él."""
        out = []
        for o in obstacles:
            if o["bottom"] < self.cfg.obstacle_bottom:
                continue
            if target is not None:
                if o["y"] <= target["y"]:
                    continue  # está detrás del objetivo
                # x de la recta (centro inferior → base del objetivo) a la altura del obstáculo
                t = (h - o["y"]) / max(h - target["y"], 1)
                line_x = w / 2 + t * (target["x"] - w / 2)
            else:
                line_x = w / 2
            gap = max(o["x1"] - line_x, line_x - o["x2"], 0.0)  # 0 si la recta cruza la caja
            if gap / (w / 2) < self.cfg.corridor:
                out.append(o)
        return out

    def _repel(self, blockers, target, w, h):
        """Término de giro que aparta el carro de los obstáculos del camino."""
        rep = 0.0
        for o in blockers:
            ox = _norm_x(o["x"], w)
            if target is not None and abs(ox) < 0.05:
                side = 1.0 if target["x"] >= o["x"] else -1.0
            else:
                side = -math.copysign(1.0, ox)
            rep += side * self.cfg.k_repel * o["bottom"]
        return rep

    def _held_w(self, now):
        """Giro del último frame, solo durante `w_hold` s (luego recto hasta el próximo frame)."""
        return self.cmd[1] if now - self.cmd_t < self.cfg.w_hold else 0.0

    def step(self, dets, frame_w, frame_h, now=None, t_frame=None):
        """Un paso de control (llamar a ~20 Hz).

        `dets` viene de `get_detections`/`classify_detections`, o es None si no hay
        frame nuevo desde el paso anterior: los estados con tiempo siguen avanzando y
        APPROACH repite el último comando. `t_frame` = hora en que se tomó el frame.
        """
        c = self.cfg
        now = time.monotonic() if now is None else now
        fresh = dets is not None
        dt = 0.05
        if fresh:
            t_frame = now if t_frame is None else t_frame
            if self.last_t is not None:
                dt = max(now - self.last_t, 1e-3)
            self.last_t = self.last_fresh = now
            targets = [d for d in dets if d["color"] == c.target_color]
            obstacles = [d for d in dets if d["color"] != c.target_color]
            self.seen = (targets, obstacles)
        else:
            targets, obstacles = [], []
        v = w = 0.0
        blockers = []

        if self.state == VERIFY:
            el = now - self.t_state
            looked = fresh and t_frame >= self.t_state + c.verify_settle
            if looked or el > c.verify_max:
                still = [t for t in targets if abs(_norm_x(t["x"], frame_w)) < 0.4]
                if still and self.retries < c.max_retries:
                    self.retries += 1
                    self._enter(APPROACH, now)
                else:
                    if not self.aborted and not still:
                        self.popped += 1
                    self.retries = 0
                    self.aborted = False
                    self.last_near = False
                    # SEARCH ve en este mismo frame si hay otro objetivo y va directo
                    self._enter(DONE if self.popped >= c.pops_goal else SEARCH, now)

        if self.state in (SEARCH, WANDER):
            if fresh and targets:
                self.confirm += 1
                if self.confirm >= c.confirm_frames:
                    self._enter(APPROACH, now)
            else:
                if fresh:
                    self.confirm = 0
                if self.state == SEARCH:  # gira un pulso, para y mira con un frame quieto
                    if now >= self.pulse_end:
                        looked = fresh and t_frame >= self.pulse_end + c.search_settle
                        if looked or now - self.pulse_end > c.search_wait_max:
                            if self.pulses >= c.search_pulses:
                                self._enter(WANDER, now)
                            else:
                                self.pulses += 1
                                self.pulse_end = now + c.search_on
                    if self.state == SEARCH and now < self.pulse_end:
                        w = self.search_dir * c.w_search
                if self.state == WANDER:  # avanzar un tramo para cambiar de punto de vista
                    if fresh:
                        blockers = self._blocking(obstacles, None, frame_w, frame_h)
                    if now - self.t_state > c.t_wander or blockers:
                        self._enter(SEARCH, now)
                    else:
                        v = c.v_wander

        if self.state == APPROACH:
            if fresh:
                free = [t for t in targets if not self._blocking(obstacles, t, frame_w, frame_h)]
                raw = pick_target(free or targets, frame_w, frame_h, "closest_prev", self.filter.pos)
                target, held = self.filter.update(raw)
                if target is None:
                    self._enter(COMMIT if self.last_near else SEARCH, now)  # perdido muy cerca → embestir
                else:
                    self.target = target
                    dx = _norm_x(target["x"], frame_w)
                    self.search_dir = 1 if dx >= 0 else -1
                    self.last_near = self._is_near(target)
                    blockers = self._blocking(obstacles, target, frame_w, frame_h)
                    aw = self.pid.update(dx, dt) + self._repel(blockers, target, frame_w, frame_h)
                    av = 0.0
                    if abs(dx) < c.align_dx:
                        av = c.v_min + (c.v_max - c.v_min) * (1 - abs(dx) / c.align_dx)
                    self.cmd, self.cmd_t = (av, aw), now
                    if self.last_near and abs(dx) < c.commit_dx and not blockers and not held:
                        self._enter(COMMIT, now)
            if self.state == APPROACH and now - self.last_fresh < c.stale_stop:
                v, w = self.cmd[0], self._held_w(now)

        if self.state == COMMIT:
            if fresh:
                blockers = [o for o in self._blocking(obstacles, None, frame_w, frame_h)
                            if self._is_near(o)]
                if targets:  # corrección fina mientras se siga viendo
                    t = pick_target(targets, frame_w, frame_h, "closest_center")
                    self.cmd, self.cmd_t = (c.v_commit, 0.5 * c.kp * _norm_x(t["x"], frame_w)), now
            if blockers or now - self.t_state > c.t_commit:
                self.aborted = bool(blockers)
                self._enter(BACKOFF, now)
            else:
                v, w = c.v_commit, self._held_w(now)

        if self.state == BACKOFF:
            if now - self.t_state > c.t_back:
                self._enter(VERIFY, now)
            else:
                v = -c.v_back

        v = float(np.clip(v, -100, 100))
        w = float(np.clip(w, -100, 100))
        return {"v": v, "w": w, "state": self.state, "popped": self.popped, "fresh": fresh,
                "target": self.target if self.state == APPROACH else None,
                "targets": self.seen[0], "obstacles": self.seen[1], "blockers": blockers}


# %% [transports]
class DriveSender:
    """Envía (v, w) si cambia o por latido; el sketch para si no recibe nada."""

    def __init__(self, transport, heartbeat_s=0.1):
        self.transport = transport
        self.heartbeat_s = heartbeat_s
        self.last = None
        self.last_t = 0.0

    def send(self, v, w):
        cmd = (int(round(v)), int(round(w)))
        now = time.monotonic()
        if cmd != self.last or now - self.last_t >= self.heartbeat_s:
            self.transport.drive(*cmd)
            self.last, self.last_t = cmd, now
            return True
        return False


class PrintDrive:
    def drive(self, v, w):
        print(f"→ MCU drive v={v:+4d} w={w:+4d}")

    def raw_pct(self, left, right):
        print(f"→ MCU raw_pct L={left:+d} R={right:+d}")

    def set_cal(self, idx, value):
        print(f"→ MCU set_cal {CAL_KEYS[idx]}={value}")

    def set_running(self, on):
        print(f"→ MCU set_running {int(on)}")

    def get_running(self):
        return 1


class BridgeDrive:
    """UNO Q: funciones del sketch vía Bridge RPC."""

    def __init__(self):
        from arduino.app_utils import Bridge
        self.bridge = Bridge

    def drive(self, v, w):
        self.bridge.call("drive", int(v), int(w))

    def raw_pct(self, left, right):
        self.bridge.call("raw_pct", int(left), int(right))

    def set_cal(self, idx, value):
        self.bridge.call("set_cal", int(idx), int(value))

    def set_running(self, on):
        self.bridge.call("set_running", int(bool(on)))

    def get_running(self):
        return int(self.bridge.call("get_running"))


# Orden de los índices de `set_cal` (igual que CAL[] en sketch.ino)
CAL_KEYS = ("GAIN_IZQ", "GAIN_DER", "MIN_IZQ", "MIN_DER")


def send_calibration(transport, cal: dict):
    for i, k in enumerate(CAL_KEYS):
        if k in cal:
            transport.set_cal(i, int(round(cal[k])))


# %% [overlay]
_BGR = {"rojo": (0, 0, 255), "verde": (0, 200, 0), "azul": (255, 80, 0), None: (160, 160, 160)}


def draw_car_overlay(frame, out, cfg: CarConfig, fps=None, perception=None):
    img = frame.copy()
    h, w = img.shape[:2]
    cv2.line(img, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
    ny = int(cfg.near_bottom * h)
    cv2.line(img, (0, ny), (w, ny), (0, 255, 255), 1)
    cw = int(cfg.corridor * w / 2)
    cv2.rectangle(img, (w // 2 - cw, int(cfg.obstacle_bottom * h)), (w // 2 + cw, h), (255, 255, 255), 1)

    for d in out["targets"] + out["obstacles"]:
        is_t = d in out["targets"]
        col = _BGR[d["color"]]
        cv2.rectangle(img, (int(d["x1"]), int(d["y1"])), (int(d["x2"]), int(d["y2"])), col, 3 if is_t else 1)
        cv2.putText(img, f"{d['label']} {d['confidence']:.2f}", (int(d["x1"]) + 4, int(d["y2"]) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
    for b in out["blockers"]:
        cv2.drawMarker(img, (int(b["x"]), int(b["y"])), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 24, 3)
    if out["target"] is not None:
        p = (int(out["target"]["x"]), int(out["target"]["y"]))
        cv2.arrowedLine(img, (w // 2, h), p, (0, 255, 255), 2, tipLength=0.05)

    cv2.putText(img, f"{out['state']}  {cfg.target_color}  {out['popped']}/{cfg.pops_goal}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(img, f"v={out['v']:+.0f} w={out['w']:+.0f}", (10, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    if perception:
        cv2.putText(img, perception, (10, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    if fps is not None:
        cv2.putText(img, f"{fps:.1f} FPS", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return img

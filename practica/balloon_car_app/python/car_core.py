"""Carro revienta-globos: color → selección de objetivo → control (v, w) → MCU.

Usa `fomo_core.py` para la inferencia y los centroides. Aquí va todo lo
específico del reto:

- `verify_color`: confirma con HSV el color de cada centroide FOMO y mide el
  área del blob (FOMO no da tamaño; el área sirve como estimación de distancia).
- `BalloonController`: máquina de estados SEARCH → APPROACH → COMMIT → BACKOFF
  → VERIFY → (SEARCH | DONE) con un PID sobre la desviación horizontal.
- `DriveSender` + transportes: envían (v, w) en [-100, 100] al sketch.

Convenciones: v > 0 = adelante; w > 0 = girar a la DERECHA (horario visto
desde arriba). El sketch mezcla L = v + w, R = v − w.
"""
# %% [imports]
import math
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from fomo_core import CentroidFilter, compute_deviation, pick_target

COLORS = ("rojo", "verde", "azul")

# Palabras que identifican el color en la etiqueta del modelo FOMO
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
    use_hsv: bool = True            # confirmar el color con HSV
    # Rangos HSV de OpenCV (H 0-179). El rojo tiene dos rangos por el "wrap" del tono.
    hsv_ranges: dict = field(default_factory=lambda: {
        "rojo": [((0, 90, 60), (10, 255, 255)), ((170, 90, 60), (179, 255, 255))],
        "verde": [((40, 70, 50), (85, 255, 255))],
        "azul": [((95, 90, 50), (130, 255, 255))],
    })
    patch_frac: float = 0.10        # lado del parche HSV (fracción del ancho)
    min_color_frac: float = 0.15    # fracción mínima de píxeles del color en el parche

    # --- Filtro del objetivo ---
    smoothing: float = 0.6
    hold_frames: int = 3
    confirm_frames: int = 2         # frames seguidos viendo el objetivo para salir de SEARCH

    # --- PID sobre dx_norm ([-1, 1]) → w ---
    kp: float = 70.0
    ki: float = 15.0
    kd: float = 8.0
    i_limit: float = 25.0           # anti-windup (en unidades de w)
    w_max: float = 60.0

    # --- Velocidades (unidades -100..100) ---
    v_max: float = 60.0
    v_min: float = 25.0
    align_dx: float = 0.35          # |dx| mayor → girar en sitio (v = 0)
    v_commit: float = 70.0
    v_back: float = 50.0
    w_search: float = 35.0

    # --- Cercanía (el globo está en el suelo: más cerca = más abajo en la imagen) ---
    near_y: float = 0.70            # cy/H a partir del cual está "cerca"
    near_area: float = 0.06         # área del blob / área del frame
    commit_dx: float = 0.15         # |dx| máximo para embestir

    # --- Tiempos (s) ---
    t_commit: float = 1.0
    t_back: float = 0.7
    t_verify: float = 0.5
    search_on: float = 0.15         # pulso de giro en SEARCH
    search_off: float = 0.20        # pausa para que la cámara no salga movida
    search_spin_s: float = 9.0      # tras esto sin ver nada → WANDER
    t_wander: float = 1.2
    max_retries: int = 2

    # --- Evitar globos de otro color ---
    corridor: float = 0.30          # medio ancho del corredor (fracción de media imagen)
    obstacle_y: float = 0.55        # obstáculos más abajo de esto (cy/H) cuentan
    k_repel: float = 45.0

    hfov_deg: float = 60.0
    vfov_deg: float = 45.0


# %% [color]
def label_to_color(label):
    """Color de una etiqueta FOMO ('globo_rojo', 'red', ...) o None si no lo indica."""
    s = label.lower()
    for color, words in _LABEL_KEYWORDS.items():
        if any(w in s for w in words):
            return color
    return None


def color_mask(hsv, color, cfg: CarConfig):
    mask = None
    for lo, hi in cfg.hsv_ranges[color]:
        m = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    return mask


def verify_color(frame_bgr, cx, cy, cfg: CarConfig):
    """Color dominante alrededor de (cx, cy) y área relativa de su blob.

    Devuelve (color | None, area_frac). El área se mide en una ventana 3× el
    parche, tomando el componente conexo que contiene (o está más cerca de) el centro.
    """
    h, w = frame_bgr.shape[:2]
    half = max(12, int(cfg.patch_frac * w / 2))
    x0, x1 = max(0, int(cx) - half), min(w, int(cx) + half)
    y0, y1 = max(0, int(cy) - half), min(h, int(cy) + half)
    if x1 <= x0 or y1 <= y0:
        return None, 0.0
    hsv = cv2.cvtColor(frame_bgr[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    fracs = {c: cv2.countNonZero(color_mask(hsv, c, cfg)) / hsv[..., 0].size for c in COLORS}
    color = max(fracs, key=fracs.get)
    if fracs[color] < cfg.min_color_frac:
        return None, 0.0

    big = 3 * half
    X0, X1 = max(0, int(cx) - big), min(w, int(cx) + big)
    Y0, Y1 = max(0, int(cy) - big), min(h, int(cy) + big)
    mask = color_mask(cv2.cvtColor(frame_bgr[Y0:Y1, X0:X1], cv2.COLOR_BGR2HSV), color, cfg)
    n, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    if n <= 1:
        return color, 0.0
    px, py = int(cx) - X0, int(cy) - Y0
    lab = labels[min(py, labels.shape[0] - 1), min(px, labels.shape[1] - 1)]
    if lab == 0:  # el centro no cae en el blob: el componente más cercano
        d = [(cents[i][0] - px) ** 2 + (cents[i][1] - py) ** 2 for i in range(1, n)]
        lab = 1 + int(np.argmin(d))
    return color, stats[lab, cv2.CC_STAT_AREA] / (w * h)


def classify_detections(frame_bgr, centroids, cfg: CarConfig):
    """Añade 'color' y 'area' a cada centroide.

    Si FOMO y HSV no coinciden el color queda en None: se trata como obstáculo
    (más vale esquivar un globo dudoso que reventar uno equivocado).
    """
    dets = []
    for c in centroids:
        if c["confidence"] < cfg.min_confidence:
            continue
        fomo_color = label_to_color(c["label"])
        area = 0.0
        color = fomo_color
        if cfg.use_hsv and frame_bgr is not None:
            hsv_color, area = verify_color(frame_bgr, c["x"], c["y"], cfg)
            if fomo_color is None:
                color = hsv_color
            elif hsv_color != fomo_color:
                color = None
        dets.append({**c, "color": color, "area": area})
    return dets


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


# %% [controller]
SEARCH, WANDER, APPROACH, COMMIT, BACKOFF, VERIFY, DONE = (
    "SEARCH", "WANDER", "APPROACH", "COMMIT", "BACKOFF", "VERIFY", "DONE")


def _norm_x(d, w):
    return (d["x"] - w / 2) / (w / 2)


class BalloonController:
    """Decide (v, w) a partir de las detecciones clasificadas de cada frame."""

    def __init__(self, cfg: CarConfig):
        self.cfg = cfg
        self.pid = PID(cfg.kp, cfg.ki, cfg.kd, cfg.i_limit, cfg.w_max)
        self.filter = CentroidFilter(cfg.smoothing, cfg.hold_frames)
        self.popped = 0
        self.retries = 0
        self.search_dir = 1
        self.confirm = 0
        self.last_near = False
        self.aborted = False        # la última embestida se abortó por un obstáculo
        self.last_t = None
        self._enter(SEARCH, time.monotonic())

    def _enter(self, state, now):
        self.state = state
        self.t_state = now
        self.pid.reset()
        self.confirm = 0
        if state in (SEARCH, WANDER):
            self.filter = CentroidFilter(self.cfg.smoothing, self.cfg.hold_frames)

    def _is_near(self, d, h):
        return d["y"] / h >= self.cfg.near_y or d.get("area", 0.0) >= self.cfg.near_area

    def _blocking(self, obstacles, target, w, h):
        """Obstáculos más cercanos que el objetivo y dentro del corredor hacia él."""
        out = []
        for o in obstacles:
            if o["y"] / h < self.cfg.obstacle_y:
                continue
            if target is not None:
                if o["y"] <= target["y"]:
                    continue  # está detrás del objetivo
                # x de la recta (centro inferior → objetivo) a la altura del obstáculo
                t = (h - o["y"]) / max(h - target["y"], 1)
                line_x = w / 2 + t * (target["x"] - w / 2)
            else:
                line_x = w / 2
            if abs(o["x"] - line_x) / (w / 2) < self.cfg.corridor:
                out.append(o)
        return out

    def _repel(self, blockers, target, w, h):
        """Término de giro que aparta el carro de los obstáculos del corredor."""
        rep = 0.0
        for o in blockers:
            ox = _norm_x(o, w)
            closeness = o["y"] / h
            side = -math.copysign(1.0, ox) if abs(ox) > 0.05 else (
                1.0 if target is None or target["x"] >= o["x"] else -1.0)
            rep += side * self.cfg.k_repel * closeness * (1 - min(abs(ox) / self.cfg.corridor, 1))
        return rep

    def step(self, dets, frame_w, frame_h, now=None):
        """Un paso de control. `dets` viene de `classify_detections`."""
        c = self.cfg
        now = time.monotonic() if now is None else now
        dt = 0.05 if self.last_t is None else now - self.last_t
        self.last_t = now
        el = now - self.t_state

        targets = [d for d in dets if d["color"] == c.target_color]
        obstacles = [d for d in dets if d["color"] != c.target_color]
        v = w = 0.0
        target = pos = None
        blockers = []

        if self.state == DONE:
            pass

        elif self.state in (SEARCH, WANDER):
            if targets:
                self.confirm += 1
                if self.confirm >= c.confirm_frames:
                    self._enter(APPROACH, now)
            else:
                self.confirm = 0
                if self.state == SEARCH:
                    if el > c.search_spin_s:
                        self._enter(WANDER, now)
                    elif el % (c.search_on + c.search_off) < c.search_on:
                        w = self.search_dir * c.w_search
                else:  # WANDER: avanzar un tramo para cambiar de punto de vista
                    blockers = self._blocking(obstacles, None, frame_w, frame_h)
                    if el > c.t_wander or blockers:
                        self._enter(SEARCH, now)
                    else:
                        v = c.v_min

        elif self.state == APPROACH:
            free = [t for t in targets if not self._blocking(obstacles, t, frame_w, frame_h)]
            target = pick_target(free or targets, frame_w, frame_h, "closest_prev", self.filter.pos)
            pos, held = self.filter.update(target)
            if pos is None:
                if self.last_near:  # se perdió por estar demasiado cerca → embestir
                    self._enter(COMMIT, now)
                    v = c.v_commit
                else:
                    self._enter(SEARCH, now)
            else:
                ref = target or {"x": pos[0], "y": pos[1], "area": 0.0}
                dev = compute_deviation(pos, frame_w, frame_h, c.hfov_deg, c.vfov_deg)
                dx = dev["dx_norm"]
                self.search_dir = 1 if dx >= 0 else -1
                self.last_near = self._is_near(ref, frame_h)
                blockers = self._blocking(obstacles, ref, frame_w, frame_h)
                w = self.pid.update(dx, dt) + self._repel(blockers, ref, frame_w, frame_h)
                if abs(dx) < c.align_dx:
                    v = c.v_min + (c.v_max - c.v_min) * (1 - abs(dx) / c.align_dx)
                if self.last_near and abs(dx) < c.commit_dx and not blockers and not held:
                    self._enter(COMMIT, now)

        elif self.state == COMMIT:
            blockers = [o for o in self._blocking(obstacles, None, frame_w, frame_h)
                        if self._is_near(o, frame_h)]
            if blockers or el > c.t_commit:
                self.aborted = bool(blockers)
                self._enter(BACKOFF, now)
            else:
                v = c.v_commit
                if targets:  # corrección fina mientras se siga viendo
                    t = pick_target(targets, frame_w, frame_h, "closest_center")
                    w = 0.5 * c.kp * _norm_x(t, frame_w)

        elif self.state == BACKOFF:
            if el > c.t_back:
                self._enter(VERIFY, now)
            else:
                v = -c.v_back

        elif self.state == VERIFY:
            if el > c.t_verify:
                still = [t for t in targets if abs(_norm_x(t, frame_w)) < 0.4]
                if still and self.retries < c.max_retries:
                    self.retries += 1
                    self._enter(APPROACH, now)
                else:
                    if not self.aborted and not still:
                        self.popped += 1
                    self.retries = 0
                    self.aborted = False
                    self.last_near = False
                    self._enter(DONE if self.popped >= c.pops_goal else SEARCH, now)

        v = float(np.clip(v, -100, 100))
        w = float(np.clip(w, -100, 100))
        return {"v": v, "w": w, "state": self.state, "popped": self.popped,
                "target": target, "pos": pos, "targets": targets,
                "obstacles": obstacles, "blockers": blockers}


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

    def set_cal(self, idx, value):
        print(f"→ MCU set_cal {idx}={value}")

    def raw_us(self, left_us, right_us):
        print(f"→ MCU raw_us L={left_us} R={right_us}")

    def set_running(self, on):
        print(f"→ MCU set_running {int(on)}")


class BridgeDrive:
    """UNO Q: funciones del sketch vía Bridge RPC."""

    def __init__(self):
        from arduino.app_utils import Bridge
        self.bridge = Bridge

    def drive(self, v, w):
        self.bridge.call("drive", int(v), int(w))

    def set_cal(self, idx, value):
        self.bridge.call("set_cal", int(idx), int(value))

    def raw_us(self, left_us, right_us):
        self.bridge.call("raw_us", int(left_us), int(right_us))

    def set_running(self, on):
        self.bridge.call("set_running", int(bool(on)))

    def get_color(self):
        """-1 mientras no se arranca con el botón; si no, 0 rojo, 1 verde, 2 azul."""
        return int(self.bridge.call("get_color"))


# Orden de los índices de `set_cal` (igual que CAL[] en sketch.ino)
CAL_KEYS = ("STOP_L", "STOP_R", "K_L_FWD", "K_L_REV", "K_R_FWD", "K_R_REV", "DB_L", "DB_R")


def send_calibration(transport, cal: dict):
    """Envía la calibración de servos. Los K van ×100 (2.0 µs/unidad → 200)."""
    for i, k in enumerate(CAL_KEYS):
        if k in cal:
            value = cal[k] * 100 if k.startswith("K_") else cal[k]
            transport.set_cal(i, int(round(value)))


# %% [overlay]
_BGR = {"rojo": (0, 0, 255), "verde": (0, 200, 0), "azul": (255, 80, 0), None: (160, 160, 160)}


def draw_car_overlay(frame, out, cfg: CarConfig, fps=None):
    img = frame.copy()
    h, w = img.shape[:2]
    cv2.line(img, (w // 2, 0), (w // 2, h), (200, 200, 200), 1)
    ny = int(cfg.near_y * h)
    cv2.line(img, (0, ny), (w, ny), (0, 255, 255), 1)
    cw = int(cfg.corridor * w / 2)
    cv2.rectangle(img, (w // 2 - cw, int(cfg.obstacle_y * h)), (w // 2 + cw, h), (255, 255, 255), 1)

    for d in out["targets"] + out["obstacles"]:
        p = (int(d["x"]), int(d["y"]))
        is_t = d in out["targets"]
        cv2.circle(img, p, 10 if is_t else 7, _BGR[d["color"]], 2 if is_t else -1)
        cv2.putText(img, f"{d['color'] or '?'} {d['confidence']:.2f} a={d['area']:.3f}",
                    (p[0] + 10, p[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, _BGR[d["color"]], 1)
    for b in out["blockers"]:
        cv2.drawMarker(img, (int(b["x"]), int(b["y"])), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 24, 3)
    if out["pos"] is not None:
        p = (int(out["pos"][0]), int(out["pos"][1]))
        cv2.arrowedLine(img, (w // 2, h), p, (0, 255, 255), 2, tipLength=0.05)

    cv2.putText(img, f"{out['state']}  {cfg.target_color}  {out['popped']}/{cfg.pops_goal}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    cv2.putText(img, f"v={out['v']:+.0f} w={out['w']:+.0f}", (10, 52),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    if fps is not None:
        cv2.putText(img, f"{fps:.1f} FPS", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return img

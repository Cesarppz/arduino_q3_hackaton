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
- `classify_detections`: color de cada globo por HSV dentro de la caja (el modelo solo dice "globo").
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
    # El modelo detecta "globo" (una clase); el color se decide con HSV dentro de
    # la caja. Con un modelo de 3 colores también se usa HSV (su etiqueta se ignora).
    use_hsv: bool = True
    hsv_s_min: int = 25             # píxel "con color": saturación mínima (baja: el sol lava el verde)
    hsv_v_min: int = 40             # y brillo mínimo (quita sombras / base negra)
    hsv_hues: dict = field(default_factory=lambda: {   # tono OpenCV 0–179
        "rojo": [(0, 10), (165, 179)],
        "verde": [(35, 90)],
        "azul": [(95, 135)],
    })
    hsv_min_colored: float = 0.05   # fracción mínima del recorte con color para decidir
    hsv_min_prob: float = 0.65      # prob. mínima del color ganador; si no → dudoso (dataset: 0.65 = 96 % ok, 1.5 % otro color)

    # --- Seguimiento del objetivo ---
    smoothing: float = 1.0          # EMA del punto (1.0 = sin suavizar: con YOLO lento y giros entre
                                    # frames, mezclar con la posición de antes del giro sesga dx)
    hold_frames: int = 2            # frames válidos sin ver el objetivo antes de darlo por perdido
    confirm_frames: int = 1         # frames quietos viendo el objetivo para salir de SEARCH
    stale_stop: float = 2.0         # s sin percepción nueva en APPROACH → parar

    # --- Gira y mira (YOLO tarda ≈0.7–1.4 s) ---
    look_settle: float = 0.35       # un frame vale si se tomó al menos esto después del último giro
                                    # (cubre el retraso del buffer de la cámara)
    align_dx: float = 0.20          # |dx| mayor → giro corto en sitio y volver a mirar
    turn_confirm: int = 2           # frames quietos seguidos con el globo al mismo lado antes de girar
    w_align: float = 45.0           # velocidad de ese giro
    turn_gain: float = 0.35         # s de giro por unidad de dx (calibrar con CAR_MODE=spin)
    turn_min: float = 0.06
    turn_max: float = 0.30
    v_approach: float = 55.0        # avance recto con el globo centrado
    k_fine: float = 30.0            # corrección suave de giro mientras avanza

    # --- Velocidades (unidades -100..100) ---
    v_commit: float = 100.0         # embestida a tope
    v_back: float = 63.0
    w_search: float = 60.0
    v_wander: float = 50.0

    # --- Cercanía (borde inferior de la caja y ancho, en fracción del frame) ---
    near_bottom: float = 0.50       # dataset: a punto de tocar ≈ 0.6–0.7
    near_width: float = 0.38        # dataset: lejos ≈ 0.3, muy cerca ≈ 0.8–0.9
    commit_dx: float = 0.15         # |dx| máximo para embestir
    commit_confirm: int = 2         # frames quietos seguidos centrado y cerca antes de embestir

    # --- Tiempos (s) ---
    t_commit: float = 1.2
    t_back: float = 0.5
    verify_max: float = 2.5         # VERIFY sin frame quieto en este tiempo → se decide sin verlo
    search_on: float = 0.30         # pulso de giro en SEARCH (más largo = más ángulo por pulso)
    search_wait_max: float = 2.5    # máximo esperando un frame quieto tras el pulso
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


def color_probs(frame_bgr, det, cfg: CarConfig):
    """Probabilidad de cada color en la zona central de la caja (sin bordes ni la base).

    Solo votan los píxeles con color (S y V mínimos); cada uno vota por su tono.
    Devuelve ({color: prob}, fracción del recorte con color).
    """
    x1, x2 = det["x1"], det["x2"]
    y1, y2 = det["y1"], det["y2"]
    bw, bh = x2 - x1, y2 - y1
    xa, xb = int(x1 + 0.25 * bw), int(x2 - 0.25 * bw)
    ya, yb = int(y1 + 0.15 * bh), int(y2 - 0.30 * bh)
    if xb - xa < 4 or yb - ya < 4:
        return {c: 0.0 for c in COLORS}, 0.0
    hsv = cv2.cvtColor(frame_bgr[ya:yb, xa:xb], cv2.COLOR_BGR2HSV).reshape(-1, 3)
    hue = hsv[(hsv[:, 1] >= cfg.hsv_s_min) & (hsv[:, 2] >= cfg.hsv_v_min), 0]
    votes = {c: int(sum(np.count_nonzero((hue >= lo) & (hue <= hi)) for lo, hi in cfg.hsv_hues[c]))
             for c in COLORS}
    total = sum(votes.values())
    probs = {c: (votes[c] / total if total else 0.0) for c in COLORS}
    return probs, total / len(hsv)


def hsv_verdict(probs, colored, cfg: CarConfig):
    """Color más probable, o None si hay poco color o no gana con claridad."""
    color = max(probs, key=probs.get)
    if colored < cfg.hsv_min_colored or probs[color] < cfg.hsv_min_prob:
        return None
    return color


def classify_detections(frame_bgr, dets, cfg: CarConfig):
    """Color de cada globo por HSV (`use_hsv`). Un globo dudoso queda con color None y se
    trata como obstáculo: más vale esquivarlo que reventar uno equivocado."""
    if not cfg.use_hsv or frame_bgr is None:
        return dets
    out = []
    for d in dets:
        probs, colored = color_probs(frame_bgr, d, cfg)
        out.append({**d, "color": hsv_verdict(probs, colored, cfg), "probs": probs, "colored": colored})
    return out


# %% [filter]
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
        self.filter = PointFilter(cfg.smoothing, cfg.hold_frames)
        self.popped = 0
        self.retries = 0
        self.search_dir = 1
        self.confirm = 0
        self.last_near = False      # último frame válido: objetivo cerca y centrado
        self.aborted = False        # la última embestida se abortó por un obstáculo
        self.last_fresh = time.monotonic()
        self.seen = ([], [])        # (targets, obstacles) del último frame nuevo
        self.move_end = -1e9        # fin del último giro: los frames anteriores no valen
        self.note = ""
        self._enter(SEARCH, time.monotonic())

    def _enter(self, state, now):
        self.state = state
        self.t_state = now
        self.confirm = 0
        self.aligned = 0
        self.turn_seen, self.turn_side = 0, 0   # frames quietos con el globo descentrado a ese lado
        self.cmd = (0.0, 0.0)       # comando de APPROACH entre frames
        self.target = None
        if state in (SEARCH, WANDER):
            self.filter = PointFilter(self.cfg.smoothing, self.cfg.hold_frames)
        if state == SEARCH:
            self.pulses = 0
            self.pulse_end = now - self.cfg.look_settle  # primero mirar (un frame desde ya vale), luego girar

    def _still(self, t_frame, since):
        """¿El frame se tomó con el carro ya quieto (después de `since`)?"""
        return t_frame >= since + self.cfg.look_settle

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

    def step(self, dets, frame_w, frame_h, now=None, t_frame=None):
        """Un paso de control (llamar a ~20 Hz).

        `dets` viene de `get_detections` + `classify_detections`, o es None si no hay
        frame nuevo desde el paso anterior. `t_frame` = hora en que se tomó el frame.
        Girar y embestir solo se deciden con frames tomados con el carro ya quieto
        tras el último giro (gira y mira), porque YOLO llega con mucho retraso.
        `out["note"]` explica la decisión tomada con el frame (vacía si no hubo).
        """
        c = self.cfg
        now = time.monotonic() if now is None else now
        fresh = dets is not None
        self.note = ""
        if fresh:
            t_frame = now if t_frame is None else t_frame
            self.last_fresh = now
            targets = [d for d in dets if d["color"] == c.target_color]
            obstacles = [d for d in dets if d["color"] != c.target_color]
            self.seen = (targets, obstacles)
        else:
            targets, obstacles = [], []
        v = w = 0.0
        blockers = []

        if self.state == VERIFY:
            looked = fresh and self._still(t_frame, self.t_state)
            if looked or now - self.t_state > c.verify_max:
                still = [t for t in targets if abs(_norm_x(t["x"], frame_w)) < 0.4]
                if still and self.retries < c.max_retries:
                    self.retries += 1
                    self.note = f"sigue ahí → reintento {self.retries}"
                    self.move_end = -1e9
                    self._enter(APPROACH, now)
                else:
                    if not self.aborted and not still:
                        self.popped += 1
                    self.retries = 0
                    self.aborted = False
                    self.last_near = False
                    # SEARCH ve en este mismo frame si hay otro objetivo y va directo
                    stopped = self.t_state  # quieto desde que empezó VERIFY
                    self._enter(DONE if self.popped >= c.pops_goal else SEARCH, now)
                    self.pulse_end = stopped

        if self.state in (SEARCH, WANDER):
            if fresh and targets:
                if self.state == SEARCH and not self._still(t_frame, self.pulse_end):
                    # visto girando: cortar el pulso y confirmar con un frame quieto
                    self.pulse_end = min(self.pulse_end, now)
                    self.note = "visto girando → parar y mirar"
                else:
                    self.confirm += 1
                    if self.confirm >= c.confirm_frames:
                        self.move_end = -1e9
                        self._enter(APPROACH, now)
            else:
                if fresh:
                    self.confirm = 0
                if self.state == SEARCH:  # gira un pulso, para y mira con un frame quieto
                    if now >= self.pulse_end:
                        looked = fresh and self._still(t_frame, self.pulse_end)
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
            if fresh and self._still(t_frame, self.move_end):
                self._approach_frame(targets, obstacles, frame_w, frame_h, now)
                blockers = self._last_blockers
            if self.state == APPROACH and now - self.last_fresh < c.stale_stop:
                if self.cmd[0] == 0.0:  # giro en sitio: solo hasta move_end, luego quieto
                    v, w = 0.0, (self.cmd[1] if now < self.move_end else 0.0)
                else:
                    v, w = self.cmd

        if self.state == COMMIT:
            if fresh:
                blockers = [o for o in self._blocking(obstacles, None, frame_w, frame_h)
                            if self._is_near(o)]
            if blockers or now - self.t_state > c.t_commit:
                self.aborted = bool(blockers)
                if blockers:
                    self.note = "globo de otro color delante → abortar"
                self._enter(BACKOFF, now)
            else:
                v = c.v_commit  # recta: los frames llegan tarde para corregir

        if self.state == BACKOFF:
            if now - self.t_state > c.t_back:
                self._enter(VERIFY, now)
            else:
                v = -c.v_back

        v = float(np.clip(v, -100, 100))
        w = float(np.clip(w, -100, 100))
        return {"v": v, "w": w, "state": self.state, "popped": self.popped, "fresh": fresh,
                "note": self.note,
                "target": self.target if self.state == APPROACH else None,
                "targets": self.seen[0], "obstacles": self.seen[1], "blockers": blockers}

    def _approach_frame(self, targets, obstacles, frame_w, frame_h, now):
        """APPROACH con un frame quieto: girar un poco, avanzar, o apuntar y embestir."""
        c = self.cfg
        self._last_blockers = []
        free = [t for t in targets if not self._blocking(obstacles, t, frame_w, frame_h)]
        raw = pick_target(free or targets, frame_w, frame_h, "closest_prev", self.filter.pos)
        target, held = self.filter.update(raw)
        if target is None:
            if self.last_near:
                self.note = "perdido cerca y centrado → embestir"
                self._enter(COMMIT, now)
            else:
                self.note = "perdido → buscar"
                self._enter(SEARCH, now)
            return
        if held:  # no se ve en este frame: parar y volver a mirar
            self.turn_seen = self.turn_side = 0
            self.cmd = (0.0, 0.0)
            self.move_end = now
            self.note = "no se ve → parar y mirar"
            return
        self.target = target
        dx = _norm_x(target["x"], frame_w)
        self.search_dir = 1 if dx >= 0 else -1
        near = self._is_near(target)
        blockers = self._last_blockers = self._blocking(obstacles, target, frame_w, frame_h)
        prob = target.get("probs", {}).get(c.target_color)
        tag = c.target_color + (f" {prob:.2f}" if prob is not None else "")
        pos = f"dx={dx:+.2f} b={target['bottom']:.2f} w={target['width']:.2f}"
        self.last_near = near and abs(dx) < c.commit_dx
        side = 1 if dx >= 0 else -1
        if abs(dx) > c.align_dx:
            self.aligned = 0
            # dar tiempo a YOLO: girar solo si varios frames quietos lo ven al mismo lado
            self.turn_seen = self.turn_seen + 1 if self.turn_side == side else 1
            self.turn_side = side
            if self.turn_seen < c.turn_confirm:
                self.cmd = (0.0, 0.0)  # quieto: el siguiente frame ya vale
                self.note = f"{tag} {pos} → confirmar {self.turn_seen}/{c.turn_confirm}"
                return
            dur = float(np.clip(c.turn_gain * abs(dx), c.turn_min, c.turn_max))
            self.cmd = (0.0, side * c.w_align)
            self.move_end = now + dur
            self.turn_seen = self.turn_side = 0
            self.note = f"{tag} {pos} → girar {dur:.2f} s"
            return
        self.turn_seen = self.turn_side = 0
        if near:
            self.cmd = (0.0, 0.0)
            self.move_end = now  # el siguiente frame tiene que tomarse ya quieto
            self.aligned = self.aligned + 1 if abs(dx) < c.commit_dx and not blockers else 0
            self.note = f"{tag} {pos} cerca → apuntado {self.aligned}/{c.commit_confirm}"
            if self.aligned >= c.commit_confirm:
                self._enter(COMMIT, now)
        else:
            self.cmd = (c.v_approach, c.k_fine * dx + self._repel(blockers, target, frame_w, frame_h))
            self.aligned = 0
            self.note = f"{tag} {pos} → avanzar"


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
    def __init__(self, turn_sign=1):
        self.turn_sign = turn_sign

    def drive(self, v, w):
        print(f"→ MCU drive v={v:+4d} w={w * self.turn_sign:+4d}")

    def raw_pct(self, left, right):
        print(f"→ MCU raw_pct L={left:+d} R={right:+d}")

    def set_cal(self, idx, value):
        print(f"→ MCU set_cal {CAL_KEYS[idx]}={value}")

    def set_running(self, on):
        print(f"→ MCU set_running {int(on)}")

    def get_running(self):
        return 1


class BridgeDrive:
    """UNO Q: funciones del sketch vía Bridge RPC.

    `turn_sign=-1` invierte el giro si el carro gira al lado contrario del pedido
    (w > 0 debe girar a la DERECHA); así no hay que volver a cargar el sketch.
    """

    def __init__(self, turn_sign=1):
        from arduino.app_utils import Bridge
        self.bridge = Bridge
        self.turn_sign = turn_sign

    def drive(self, v, w):
        self.bridge.call("drive", int(v), int(w) * self.turn_sign)

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
        if "probs" in d:  # color por HSV: ganador (o "?" si dudoso) y su probabilidad
            best = max(d["probs"], key=d["probs"].get)
            txt = f"{d['color'] or '?'} {d['probs'][best]:.2f} ({d['confidence']:.2f})"
        else:
            txt = f"{d['label']} {d['confidence']:.2f}"
        cv2.putText(img, txt, (int(d["x1"]) + 4, int(d["y2"]) - 6),
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

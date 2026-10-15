"""Núcleo FOMO -> centroide -> desviación -> byte de flags para el MCU.

Compartido entre el notebook `practica/fomo_desviacion.ipynb` y la app del
Arduino UNO Q (`python/main.py`). Las secciones `# %% [...]` son las celdas
que el notebook muestra y explica.
"""
# %% [imports]
import math
import os
import platform
import stat
import sys
import time
import types
from dataclasses import dataclass

import cv2
import numpy as np

# edge_impulse_linux importa su módulo de audio (pyaudio) al cargarse.
# No usamos audio: si pyaudio no está instalado ponemos un módulo vacío.
try:
    import pyaudio  # noqa: F401
except ImportError:
    sys.modules["pyaudio"] = types.ModuleType("pyaudio")

from edge_impulse_linux.image import ImageImpulseRunner


# %% [config]
@dataclass
class Config:
    model_path: str = "modelo.eim"
    camera_id: int = 0
    min_confidence: float = 0.5   # descarta centroides con menor confianza
    target_label: str | None = None  # None = cualquier etiqueta
    target_strategy: str = "closest_prev"  # "confidence" | "closest_center" | "closest_prev"
    deadzone: float = 0.10        # fracción de media imagen (0.10 = ±10 % alrededor del centro)
    hysteresis: float = 0.03      # margen para salir de LEFT/RIGHT/UP/DOWN (evita oscilar)
    smoothing: float = 0.5        # EMA: peso del centroide nuevo (1.0 = sin suavizado)
    lost_hold_frames: int = 5     # frames que se mantiene la última posición si se pierde
    hfov_deg: float = 60.0        # campo de visión horizontal de la cámara
    vfov_deg: float = 45.0        # campo de visión vertical


# %% [load_model]
_ELF_MACHINES = {0x3E: "x86_64", 0xB7: "aarch64", 0x28: "armv7l"}


def eim_architecture(path):
    """Lee la cabecera ELF del .eim y devuelve su arquitectura (x86_64, aarch64...)."""
    with open(path, "rb") as f:
        header = f.read(20)
    if header[:4] != b"\x7fELF":
        raise ValueError(f"{path} no es un ejecutable ELF (¿descarga incompleta?)")
    machine = int.from_bytes(header[18:20], "little")
    return _ELF_MACHINES.get(machine, f"desconocida (0x{machine:x})")


def load_model(path, verbose=True):
    """Carga un .eim de Edge Impulse y devuelve (runner, model_info)."""
    path = os.path.abspath(path)
    if not os.path.exists(path):
        raise FileNotFoundError(f"No existe el modelo: {path}")

    arch, host = eim_architecture(path), platform.machine()
    if arch != host:
        raise RuntimeError(
            f"El .eim es para {arch} pero esta máquina es {host}. "
            f"Descarga el build 'Linux ({host})' en Edge Impulse → Deployment."
        )

    # El .eim debe ser ejecutable
    mode = os.stat(path).st_mode
    if not mode & stat.S_IXUSR:
        os.chmod(path, mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)

    runner = ImageImpulseRunner(path)
    info = runner.init()
    p = info["model_parameters"]
    if verbose:
        print(f"Proyecto    : {info['project']['owner']} / {info['project']['name']}")
        print(f"Arquitectura: {arch}")
        print(f"Tipo        : {p['model_type']}")
        print(f"Etiquetas   : {p['labels']}")
        print(f"Entrada     : {p['image_input_width']}x{p['image_input_height']}"
              f" x{p['image_channel_count']} canales, resize '{runner.resizeMode}'")
    if p["model_type"] != "constrained_object_detection":
        print(f"⚠ El modelo no es FOMO (model_type={p['model_type']}): "
              "las cajas sí tendrán tamaño real y el centroide será el centro de la caja.")
    return runner, info


# %% [model_to_frame]
def model_to_frame(x, y, frame_w, frame_h, model_w, model_h, mode):
    """Convierte (x, y) de coordenadas del modelo a coordenadas del frame original.

    Edge Impulse redimensiona el frame según el 'resize mode' configurado en Studio.
    """
    if mode in ("", "not-reported", "squash"):
        return x * frame_w / model_w, y * frame_h / model_h

    if mode == "fit-shortest":
        # Recorte central con la relación de aspecto del modelo y luego resize
        aspect = model_w / model_h
        if frame_w / frame_h > aspect:      # frame más ancho: se recortan los lados
            crop_w = int(frame_h * aspect)
            off_x, off_y = (frame_w - crop_w) // 2, 0
            scale = frame_h / model_h
        else:                               # frame más alto: se recorta arriba/abajo
            crop_h = int(frame_w / aspect)
            off_x, off_y = 0, (frame_h - crop_h) // 2
            scale = frame_w / model_w
        return off_x + x * scale, off_y + y * scale

    if mode == "fit-longest":
        # Letterbox: se escala manteniendo aspecto y se rellena con bordes negros
        scale = min(model_w / frame_w, model_h / frame_h)
        pad_x = (model_w - int(frame_w * scale)) // 2
        pad_y = (model_h - int(frame_h * scale)) // 2
        return (x - pad_x) / scale, (y - pad_y) / scale

    raise ValueError(f"Resize mode no soportado: {mode}")


# %% [centroids]
def run_inference(runner, frame_bgr):
    """Ejecuta el modelo sobre un frame BGR de OpenCV y devuelve la respuesta cruda."""
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    features, _ = runner.get_features_from_image_auto_studio_settings(rgb)
    return runner.classify(features)


def get_centroids(res, frame_w, frame_h, model_w, model_h, mode,
                  min_confidence=0.5, target_label=None):
    """Extrae los centroides FOMO en coordenadas del frame.

    FOMO solo predice *dónde* está el objeto (una rejilla 1/8 de la entrada).
    El SDK fusiona las celdas activas en `bounding_boxes`; su centro es el
    centroide. El ancho/alto NO es el tamaño del objeto, así que no se devuelve.
    """
    centroids = []
    for bb in res.get("result", {}).get("bounding_boxes", []):
        if bb["value"] < min_confidence:
            continue
        if target_label is not None and bb["label"] != target_label:
            continue
        cx, cy = model_to_frame(bb["x"] + bb["width"] / 2, bb["y"] + bb["height"] / 2,
                                frame_w, frame_h, model_w, model_h, mode)
        centroids.append({"label": bb["label"], "confidence": bb["value"], "x": cx, "y": cy})
    return centroids


# %% [pick_target]
def pick_target(centroids, frame_w, frame_h, strategy="confidence", prev=None):
    """Elige un único objetivo cuando FOMO devuelve varios centroides.

    - "confidence": el de mayor confianza.
    - "closest_center": el más cercano al centro de la imagen (menos movimiento).
    - "closest_prev": el más cercano al objetivo anterior (no salta entre objetos).
    """
    if not centroids:
        return None
    if strategy == "closest_prev" and prev is not None:
        ref = prev
    elif strategy in ("closest_center", "closest_prev"):
        ref = (frame_w / 2, frame_h / 2)
    else:
        return max(centroids, key=lambda c: c["confidence"])
    return min(centroids, key=lambda c: (c["x"] - ref[0]) ** 2 + (c["y"] - ref[1]) ** 2)


# %% [filter]
class CentroidFilter:
    """Suaviza el centroide (media móvil exponencial) y tolera pérdidas breves.

    FOMO a veces "parpadea" (pierde el objeto 1-2 frames). En vez de mandar
    "no detectado" al motor, se mantiene la última posición `hold_frames` frames.
    """

    def __init__(self, smoothing=0.5, hold_frames=5):
        self.alpha = smoothing
        self.hold_frames = hold_frames
        self.pos = None
        self.missed = 0

    def update(self, target):
        """Devuelve ((x, y), held) o (None, False) si el objetivo se perdió."""
        if target is not None:
            new = (target["x"], target["y"])
            if self.pos is None or self.missed > 0:
                self.pos = new
            else:
                a = self.alpha
                self.pos = (a * new[0] + (1 - a) * self.pos[0],
                            a * new[1] + (1 - a) * self.pos[1])
            self.missed = 0
            return self.pos, False

        if self.pos is not None and self.missed < self.hold_frames:
            self.missed += 1
            return self.pos, True

        self.pos = None
        return None, False


# %% [deviation]
def compute_deviation(centroid, frame_w, frame_h, hfov_deg=60.0, vfov_deg=45.0):
    """Desviación del centroide respecto al centro de la cámara.

    Ejes: x positivo a la derecha, y positivo hacia arriba.
    """
    cx, cy = centroid
    center_x, center_y = frame_w / 2, frame_h / 2

    dx_px = cx - center_x
    dy_px = center_y - cy           # invertimos y: en la imagen crece hacia abajo

    dx_norm = dx_px / center_x      # [-1, 1]
    dy_norm = dy_px / center_y

    # Modelo pinhole: ángulo = atan(desplazamiento_normalizado * tan(FOV/2))
    dx_deg = math.degrees(math.atan(dx_norm * math.tan(math.radians(hfov_deg / 2))))
    dy_deg = math.degrees(math.atan(dy_norm * math.tan(math.radians(vfov_deg / 2))))

    return {"dx_px": dx_px, "dy_px": dy_px,
            "dx_norm": dx_norm, "dy_norm": dy_norm,
            "dx_deg": dx_deg, "dy_deg": dy_deg}


# %% [encoder]
DETECTED = 1 << 0   # hay objetivo
LEFT     = 1 << 1   # objeto a la izquierda del centro
RIGHT    = 1 << 2   # objeto a la derecha
UP       = 1 << 3   # objeto arriba
DOWN     = 1 << 4   # objeto abajo
CENTERED = 1 << 5   # dentro de la zona muerta en ambos ejes
HOLD     = 1 << 6   # posición estimada (objeto perdido, se usa la última conocida)

FLAG_NAMES = {DETECTED: "DETECTED", LEFT: "LEFT", RIGHT: "RIGHT", UP: "UP",
              DOWN: "DOWN", CENTERED: "CENTERED", HOLD: "HOLD"}


def _axis_state(value, state, deadzone, hysteresis):
    """-1, 0 o +1 con histéresis: se entra a ±1 al pasar `deadzone` y se vuelve
    a 0 solo al bajar de `deadzone - hysteresis`."""
    if abs(value) > deadzone:
        return 1 if value > 0 else -1
    if state != 0 and abs(value) > deadzone - hysteresis:
        return state
    return 0


class DirectionEncoder:
    """Convierte la desviación en un byte de flags para el microcontrolador."""

    def __init__(self, deadzone=0.10, hysteresis=0.03):
        self.deadzone = deadzone
        self.hysteresis = hysteresis
        self.sx = 0
        self.sy = 0

    def encode(self, deviation, held=False):
        if deviation is None:
            self.sx = self.sy = 0
            return 0
        self.sx = _axis_state(deviation["dx_norm"], self.sx, self.deadzone, self.hysteresis)
        self.sy = _axis_state(deviation["dy_norm"], self.sy, self.deadzone, self.hysteresis)

        flags = DETECTED
        flags |= LEFT if self.sx < 0 else RIGHT if self.sx > 0 else 0
        flags |= DOWN if self.sy < 0 else UP if self.sy > 0 else 0
        if self.sx == 0 and self.sy == 0:
            flags |= CENTERED
        if held:
            flags |= HOLD
        return flags


def decode_flags(flags):
    """Texto legible de un byte de flags, p. ej. 'DETECTED|LEFT|UP'."""
    names = [n for bit, n in FLAG_NAMES.items() if flags & bit]
    return "|".join(names) if names else "NONE"


# %% [pipeline]
class Pipeline:
    """Todo el proceso para un frame: inferencia → centroide → filtro → desviación → flags."""

    def __init__(self, runner, cfg: Config):
        self.runner = runner
        self.cfg = cfg
        self.model_w, self.model_h = runner.dim
        self.filter = CentroidFilter(cfg.smoothing, cfg.lost_hold_frames)
        self.encoder = DirectionEncoder(cfg.deadzone, cfg.hysteresis)

    def process(self, frame_bgr):
        h, w = frame_bgr.shape[:2]
        t0 = time.perf_counter()
        res = run_inference(self.runner, frame_bgr)
        return self.process_result(res, w, h, time.perf_counter() - t0)

    def process_result(self, res, w, h, infer_s=0.0):
        """Igual que process() pero a partir de una respuesta ya calculada (útil para pruebas)."""
        c = self.cfg
        self.encoder.deadzone, self.encoder.hysteresis = c.deadzone, c.hysteresis
        centroids = get_centroids(res, w, h, self.model_w, self.model_h, self.runner.resizeMode,
                                  c.min_confidence, c.target_label)
        target = pick_target(centroids, w, h, c.target_strategy, self.filter.pos)
        pos, held = self.filter.update(target)
        deviation = compute_deviation(pos, w, h, c.hfov_deg, c.vfov_deg) if pos else None
        flags = self.encoder.encode(deviation, held)
        return {"centroids": centroids, "target": target, "pos": pos, "held": held,
                "deviation": deviation, "flags": flags,
                "timing": res.get("timing", {}), "infer_s": infer_s}


# %% [transports]
class FlagSender:
    """Envía el byte solo cuando cambia, más un 'latido' periódico.

    El latido permite que el MCU detenga el motor si deja de recibir datos
    (p. ej. si se cae la app de Python).
    """

    def __init__(self, transport, heartbeat_s=0.2):
        self.transport = transport
        self.heartbeat_s = heartbeat_s
        self.last = None
        self.last_t = 0.0

    def send(self, flags):
        now = time.monotonic()
        if flags != self.last or now - self.last_t >= self.heartbeat_s:
            self.transport.write(flags)
            self.last, self.last_t = flags, now
            return True
        return False


class PrintTransport:
    """Para depurar: imprime el byte en vez de enviarlo."""

    def write(self, flags):
        print(f"→ MCU 0b{flags:08b} ({decode_flags(flags)})")


class SerialTransport:
    """Arduino clásico por USB: envía 1 byte crudo (ver arduino_serial_flags.ino)."""

    def __init__(self, port="/dev/ttyACM0", baud=115200):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0)
        time.sleep(2)  # el Arduino se reinicia al abrir el puerto

    def write(self, flags):
        self.ser.write(bytes([flags]))

    def close(self):
        self.ser.close()


class BridgeTransport:
    """Arduino UNO Q: llama a `set_flags` en el sketch del STM32 vía Bridge (RPC)."""

    def __init__(self, method="set_flags"):
        from arduino.app_utils import Bridge
        self.bridge = Bridge
        self.method = method

    def write(self, flags):
        self.bridge.call(self.method, int(flags))


# %% [overlay]
def draw_overlay(frame, out, cfg: Config, fps=None):
    """Dibuja ejes, zona muerta, centroides, objetivo, desviación y el byte enviado."""
    img = frame.copy()
    h, w = img.shape[:2]
    cx, cy = w // 2, h // 2

    # Ejes centrales y zona muerta
    cv2.line(img, (cx, 0), (cx, h), (200, 200, 200), 1)
    cv2.line(img, (0, cy), (w, cy), (200, 200, 200), 1)
    dzx, dzy = int(cfg.deadzone * w / 2), int(cfg.deadzone * h / 2)
    cv2.rectangle(img, (cx - dzx, cy - dzy), (cx + dzx, cy + dzy), (255, 255, 255), 1)

    for c in out["centroids"]:
        p = (int(c["x"]), int(c["y"]))
        cv2.circle(img, p, 6, (0, 255, 0), 2)
        cv2.putText(img, f"{c['label']} {c['confidence']:.2f}", (p[0] + 8, p[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    flags, dev = out["flags"], out["deviation"]
    if out["pos"] is not None:
        p = (int(out["pos"][0]), int(out["pos"][1]))
        color = (0, 165, 255) if out["held"] else (0, 0, 255)
        cv2.circle(img, p, 8, color, -1)
        cv2.arrowedLine(img, (cx, cy), p, (0, 255, 255), 2, tipLength=0.05)
        cv2.putText(img, f"dx={dev['dx_px']:+.0f}px ({dev['dx_deg']:+.1f} deg)  "
                         f"dy={dev['dy_px']:+.0f}px ({dev['dy_deg']:+.1f} deg)",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    else:
        cv2.putText(img, "Sin deteccion", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    cv2.putText(img, f"0b{flags:08b}  {decode_flags(flags)}", (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    if fps is not None:
        cv2.putText(img, f"{fps:.1f} FPS", (10, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return img

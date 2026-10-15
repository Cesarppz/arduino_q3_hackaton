"""Data augmentation + preprocesamiento de imágenes para Edge Impulse (FOMO).

Pensado para un robot-carro con Arduino UNO Q y webcam Logitech 720p: genera
variantes con cambios de iluminación (exposición, balance de blancos, sombras,
poca luz) y de vibración (motion blur, desenfoque, pequeños giros/saltos,
rolling shutter, compresión MJPEG).

Se aplica ANTES de etiquetar: la carpeta de salida se sube a Edge Impulse
Studio (Data acquisition -> Upload data) y las cajas se dibujan ahí.

Uso:
    python augment_edge_impulse.py --input fotos/ --output dataset_aug/ \
        --copies 3 --size 320 --seed 42 --preview preview.jpg

Si la entrada tiene subcarpetas `training/` y `testing/` (export de Studio),
se respetan y `testing/` solo se preprocesa (sin augmentation).
"""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np

IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}

# %% [config]
# Probabilidad de aplicar cada augmentation y sus rangos. Ajusta aquí.
AUG_CONFIG = {
    # --- Iluminación ---
    "brightness_contrast": {"p": 0.8, "brightness": (-50, 50), "contrast": (0.6, 1.4)},
    "gamma": {"p": 0.5, "range": (0.5, 1.8)},
    "white_balance": {"p": 0.5, "gain": (0.75, 1.25)},       # ganancia R y B
    "hsv": {"p": 0.4, "hue": (-8, 8), "saturation": (0.6, 1.4)},
    "shadow": {"p": 0.4, "strength": (0.3, 0.7)},             # sombra/gradiente local
    "overexposure": {"p": 0.15, "gain": (1.3, 1.8)},
    "low_light": {"p": 0.2, "factor": (0.25, 0.55), "noise_sigma": (4, 12)},
    "sensor_noise": {"p": 0.3, "sigma": (2, 8)},
    # --- Vibración / movimiento ---
    "geometric": {
        "p": 0.7,
        "rotation_deg": 10,     # ±
        "translate": 0.10,      # fracción del tamaño, ±
        "scale": (0.85, 1.15),
        "perspective": 0.04,    # desplazamiento máx. de esquinas (fracción)
    },
    "rolling_shutter": {"p": 0.2, "shear": 0.08},
    "motion_blur": {"p": 0.5, "length": (3, 15), "horizontal_bias": 0.7},
    "defocus_blur": {"p": 0.25, "ksize": (3, 7)},
    "jpeg": {"p": 0.5, "quality": (40, 90)},
}


# %% [preprocess]
def preprocess_frame(img, size=320, clahe=False):
    """Preprocesado común a entrenamiento e inferencia.

    Redimensiona dejando el lado mayor en `size` (mantiene el aspecto) y,
    opcionalmente, aplica CLAHE en el canal L. Si usas `clahe=True` aquí,
    aplica también esta función a cada frame en `fomo_core.detect()`.
    """
    if size:
        h, w = img.shape[:2]
        scale = size / max(h, w)
        if scale != 1:
            interp = cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR
            img = cv2.resize(img, (round(w * scale), round(h * scale)), interpolation=interp)
    if clahe:
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
        img = cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)
    return img


# %% [lighting]
def _lut(fn):
    x = np.arange(256, dtype=np.float32)
    return np.clip(fn(x), 0, 255).astype(np.uint8)


def brightness_contrast(img, rng, cfg):
    alpha = rng.uniform(*cfg["contrast"])
    beta = rng.uniform(*cfg["brightness"])
    return cv2.convertScaleAbs(img, alpha=alpha, beta=beta)


def gamma(img, rng, cfg):
    g = rng.uniform(*cfg["range"])
    return cv2.LUT(img, _lut(lambda x: 255.0 * (x / 255.0) ** g))


def white_balance(img, rng, cfg):
    gains = np.array([rng.uniform(*cfg["gain"]), 1.0, rng.uniform(*cfg["gain"])], np.float32)  # B, G, R
    return np.clip(img.astype(np.float32) * gains, 0, 255).astype(np.uint8)


def hsv(img, rng, cfg):
    h, s, v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    h = ((h.astype(np.int16) + int(rng.uniform(*cfg["hue"]))) % 180).astype(np.uint8)
    s = np.clip(s.astype(np.float32) * rng.uniform(*cfg["saturation"]), 0, 255).astype(np.uint8)
    return cv2.cvtColor(cv2.merge((h, s, v)), cv2.COLOR_HSV2BGR)


def shadow(img, rng, cfg):
    """Oscurece una zona con un gradiente lineal o una elipse difusa."""
    h, w = img.shape[:2]
    strength = rng.uniform(*cfg["strength"])
    if rng.random() < 0.5:
        # Gradiente lineal en dirección aleatoria (luz lateral / sombra de un objeto)
        ang = rng.uniform(0, 2 * math.pi)
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        proj = (xx - w / 2) * math.cos(ang) + (yy - h / 2) * math.sin(ang)
        proj = (proj - proj.min()) / (np.ptp(proj) + 1e-6)
        mask = 1.0 - strength * proj
    else:
        # Mancha elíptica difusa (sombra o reflejo)
        mask = np.zeros((h, w), np.float32)
        center = (int(rng.uniform(0, w)), int(rng.uniform(0, h)))
        axes = (int(rng.uniform(0.15, 0.5) * w), int(rng.uniform(0.15, 0.5) * h))
        cv2.ellipse(mask, center, axes, rng.uniform(0, 180), 0, 360, 1.0, -1)
        k = max(3, (min(h, w) // 4) | 1)
        mask = cv2.GaussianBlur(mask, (k, k), 0)
        sign = -1 if rng.random() < 0.75 else 1  # 75 % sombra, 25 % zona más iluminada
        mask = 1.0 + sign * strength * mask
    return np.clip(img.astype(np.float32) * mask[..., None], 0, 255).astype(np.uint8)


def overexposure(img, rng, cfg):
    return np.clip(img.astype(np.float32) * rng.uniform(*cfg["gain"]), 0, 255).astype(np.uint8)


def low_light(img, rng, cfg):
    """Imagen oscura con el ruido que mete la cámara al subir la ganancia."""
    out = img.astype(np.float32) * rng.uniform(*cfg["factor"])
    out = rng.poisson(np.clip(out, 0, None)).astype(np.float32)
    out += rng.normal(0, rng.uniform(*cfg["noise_sigma"]), img.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


def sensor_noise(img, rng, cfg):
    noise = rng.normal(0, rng.uniform(*cfg["sigma"]), img.shape)
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)


# %% [vibration]
def geometric(img, rng, cfg):
    """Rotación + escala + traslación + perspectiva leve en una sola homografía."""
    h, w = img.shape[:2]
    ang = rng.uniform(-cfg["rotation_deg"], cfg["rotation_deg"])
    scale = rng.uniform(*cfg["scale"])
    m = np.vstack([cv2.getRotationMatrix2D((w / 2, h / 2), ang, scale), [0, 0, 1]])
    t = cfg["translate"]
    m[0, 2] += rng.uniform(-t, t) * w
    m[1, 2] += rng.uniform(-t, t) * h

    p = cfg["perspective"]
    src = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst = src + rng.uniform(-p, p, (4, 2)).astype(np.float32) * [w, h]
    persp = cv2.getPerspectiveTransform(src, dst.astype(np.float32))
    return cv2.warpPerspective(img, persp @ m, (w, h), borderMode=cv2.BORDER_REFLECT)


def rolling_shutter(img, rng, cfg):
    """Shear horizontal: la cámara se mueve mientras el sensor lee las filas."""
    h, w = img.shape[:2]
    s = rng.uniform(-cfg["shear"], cfg["shear"])
    m = np.float32([[1, s, -s * h / 2], [0, 1, 0]])
    return cv2.warpAffine(img, m, (w, h), borderMode=cv2.BORDER_REFLECT)


def motion_blur(img, rng, cfg):
    length = int(rng.integers(cfg["length"][0], cfg["length"][1] + 1))
    if rng.random() < cfg["horizontal_bias"]:
        ang = rng.uniform(-20, 20)          # avance/giro del carro
    else:
        ang = rng.uniform(0, 180)           # vibración en cualquier dirección
    k = np.zeros((length, length), np.float32)
    k[length // 2, :] = 1.0
    rot = cv2.getRotationMatrix2D(((length - 1) / 2, (length - 1) / 2), ang, 1.0)
    k = cv2.warpAffine(k, rot, (length, length))
    k /= max(k.sum(), 1e-6)
    return cv2.filter2D(img, -1, k)


def defocus_blur(img, rng, cfg):
    k = int(rng.integers(cfg["ksize"][0], cfg["ksize"][1] + 1)) | 1
    return cv2.GaussianBlur(img, (k, k), 0)


def jpeg(img, rng, cfg):
    q = int(rng.integers(cfg["quality"][0], cfg["quality"][1] + 1))
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, q])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR) if ok else img


# Orden de aplicación: primero geometría (la cámara se mueve), luego
# iluminación (la escena), luego sensor (blur, ruido) y por último la compresión.
PIPELINE = [
    ("geometric", geometric),
    ("rolling_shutter", rolling_shutter),
    ("brightness_contrast", brightness_contrast),
    ("gamma", gamma),
    ("white_balance", white_balance),
    ("hsv", hsv),
    ("shadow", shadow),
    ("overexposure", overexposure),
    ("low_light", low_light),
    ("motion_blur", motion_blur),
    ("defocus_blur", defocus_blur),
    ("sensor_noise", sensor_noise),
    ("jpeg", jpeg),
]


def augment(img, rng, hflip=False):
    """Aplica una combinación aleatoria de augmentations. Garantiza al menos una."""
    out = img
    applied = []
    for name, fn in PIPELINE:
        cfg = AUG_CONFIG[name]
        if rng.random() < cfg["p"]:
            out = fn(out, rng, cfg)
            applied.append(name)
    if not applied:
        name, fn = PIPELINE[int(rng.integers(len(PIPELINE)))]
        out = fn(out, rng, AUG_CONFIG[name])
        applied.append(name)
    if hflip and rng.random() < 0.5:
        out = cv2.flip(out, 1)
        applied.append("hflip")
    return out, applied


# %% [io]
def find_images(root):
    return sorted(p for p in root.rglob("*") if p.is_file() and p.suffix.lower() in IMG_EXTS)


def is_testing(rel_path):
    return len(rel_path.parts) > 1 and rel_path.parts[0].lower() == "testing"


def make_preview(groups, path, tile=200, max_rows=6):
    """Cuadrícula: cada fila es un original seguido de sus variantes."""
    rows = []
    for imgs in groups[:max_rows]:
        tiles = []
        for im in imgs:
            h, w = im.shape[:2]
            s = tile / max(h, w)
            im = cv2.resize(im, (round(w * s), round(h * s)))
            canvas = np.zeros((tile, tile, 3), np.uint8)
            y0, x0 = (tile - im.shape[0]) // 2, (tile - im.shape[1]) // 2
            canvas[y0:y0 + im.shape[0], x0:x0 + im.shape[1]] = im
            tiles.append(canvas)
        rows.append(tiles)
    width = max(len(r) for r in rows)
    blank = np.zeros((tile, tile, 3), np.uint8)
    grid = np.vstack([np.hstack(r + [blank] * (width - len(r))) for r in rows])
    cv2.imwrite(str(path), grid)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--input", required=True, type=Path, help="carpeta con imágenes (o export de Studio)")
    ap.add_argument("--output", required=True, type=Path, help="carpeta de salida para subir a Studio")
    ap.add_argument("--copies", type=int, default=3, help="variantes aumentadas por imagen (default 3)")
    ap.add_argument("--size", type=int, default=320, help="lado mayor tras el resize; 0 = sin resize")
    ap.add_argument("--seed", type=int, default=None, help="semilla para resultados reproducibles")
    ap.add_argument("--clahe", action="store_true", help="aplicar CLAHE (¡también en inferencia!)")
    ap.add_argument("--hflip", action="store_true", help="permitir espejo horizontal")
    ap.add_argument("--quality", type=int, default=95, help="calidad JPEG de los archivos de salida")
    ap.add_argument("--preview", type=Path, default=None, help="guardar cuadrícula de ejemplo en esta ruta")
    args = ap.parse_args()

    if not args.input.is_dir():
        ap.error(f"no existe la carpeta de entrada: {args.input}")
    rng = np.random.default_rng(args.seed)
    images = find_images(args.input)
    if not images:
        ap.error(f"no se encontraron imágenes en {args.input}")

    n_read = n_written = n_skipped = 0
    preview_groups = []
    for src in images:
        rel = src.relative_to(args.input)
        img = cv2.imread(str(src), cv2.IMREAD_COLOR)
        if img is None:
            print(f"[aviso] no se pudo leer {rel}, se salta")
            n_skipped += 1
            continue
        n_read += 1

        out_dir = args.output / rel.parent
        out_dir.mkdir(parents=True, exist_ok=True)
        base = preprocess_frame(img, args.size, args.clahe)
        params = [cv2.IMWRITE_JPEG_QUALITY, args.quality]
        cv2.imwrite(str(out_dir / f"{src.stem}.jpg"), base, params)
        n_written += 1

        group = [base]
        if not is_testing(rel):  # testing/ se deja sin augmentation
            for k in range(1, args.copies + 1):
                aug, _ = augment(base, rng, args.hflip)
                cv2.imwrite(str(out_dir / f"{src.stem}.aug{k}.jpg"), aug, params)
                n_written += 1
                group.append(aug)
        if args.preview and len(group) > 1 and len(preview_groups) < 6:
            preview_groups.append(group)

    if args.preview and preview_groups:
        args.preview.parent.mkdir(parents=True, exist_ok=True)
        make_preview(preview_groups, args.preview)
        print(f"Preview guardado en {args.preview}")

    print(f"Imágenes leídas   : {n_read}")
    print(f"Imágenes escritas : {n_written}  (en {args.output})")
    print(f"Saltadas          : {n_skipped}")
    if args.clahe:
        print("Recuerda: usaste --clahe, aplica preprocess_frame(..., clahe=True) también en inferencia.")


if __name__ == "__main__":
    main()

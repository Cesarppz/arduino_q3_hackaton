"""Evalúa el modelo YOLO (.eim) y la lógica del carro con un export de Edge Impulse.

    python eval_dataset.py <carpeta_export> --model model.eim [--out overlays/] [--color rojo]
    python eval_dataset.py <carpeta_export> --brick [--out overlays/]   (UNO Q, brick de App Lab)
    python eval_dataset.py <carpeta_export> --gt [--out overlays/]

- Con --brick: igual, pero con el brick Object Detection de App Lab.
- Con --model: corre el .eim sobre cada imagen y compara con las cajas
  etiquetadas de info.labels (acierto = misma etiqueta e IoU >= 0.3).
  El .eim tiene que ser de la arquitectura de la máquina (AARCH64 en el UNO Q,
  x86_64 en la laptop).
- Con --gt: no usa el modelo; convierte las cajas etiquetadas en una respuesta
  falsa del SDK para probar get_detections + BalloonController con imágenes reales.

En todos los modos el color de cada caja se decide por HSV (classify_detections)
y se compara con la etiqueta de color del export (rojo/verde/azul): al final se
imprime la matriz de confusión para ajustar hsv_s_min / hsv_hues / hsv_min_prob.
Con un modelo de una clase ("globo"), las cajas se emparejan solo por IoU.

--out guarda cada imagen con el overlay (cajas, estado, v/w).
"""
import argparse
import json
import os

import cv2

from car_core import (COLORS, BalloonController, CarConfig, classify_detections, draw_car_overlay,
                      get_detections, label_to_color)


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union else 0.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("export")
    ap.add_argument("--model")
    ap.add_argument("--brick", action="store_true")
    ap.add_argument("--gt", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--color", default="rojo")
    ap.add_argument("--min-conf", type=float, default=0.5)
    args = ap.parse_args()
    if not (args.model or args.brick or args.gt):
        ap.error("usa --model <.eim>, --brick o --gt")

    info = json.load(open(os.path.join(args.export, "info.labels")))
    files = [f for f in info["files"] if f.get("boundingBoxes")]
    print(f"{len(files)} imágenes con cajas de {len(info['files'])}")

    detector = None
    if args.brick:
        from vision import BrickDetector
        detector = BrickDetector(args.min_conf)
    elif args.model:
        from vision import EimDetector
        detector = EimDetector(args.model)
    if args.out:
        os.makedirs(args.out, exist_ok=True)

    cfg = CarConfig(target_color=args.color, min_confidence=args.min_conf)
    tp = fp = fn = 0
    conf = {g: {p: 0 for p in COLORS + (None,)} for g in COLORS}   # [verdad][HSV]
    for f in files:
        img = cv2.imread(os.path.join(args.export, f["path"]))
        if img is None:
            continue
        h, w = img.shape[:2]
        gt = [(b["label"], (b["x"], b["y"], b["x"] + b["width"], b["y"] + b["height"]))
              for b in f["boundingBoxes"]]

        if detector:
            res = detector.infer(img)
            dets = get_detections(res, w, h, *detector.dim, detector.resize_mode, cfg.min_confidence)
        else:
            res = {"result": {"bounding_boxes": [
                {"label": b["label"], "value": 1.0, "x": b["x"], "y": b["y"],
                 "width": b["width"], "height": b["height"]} for b in f["boundingBoxes"]]}}
            dets = get_detections(res, w, h, w, h, "squash", cfg.min_confidence)
        dets = classify_detections(img, dets, cfg)

        matched = set()
        for d in dets:
            box = (d["x1"], d["y1"], d["x2"], d["y2"])
            one_class = label_to_color(d["label"]) is None   # modelo "globo": solo IoU
            hit = next((i for i, (lab, g) in enumerate(gt)
                        if i not in matched and (one_class or lab == d["label"]) and iou(box, g) >= 0.3),
                       None)
            if hit is None:
                fp += 1
            else:
                tp += 1
                matched.add(hit)
                g = label_to_color(gt[hit][0])
                if g:
                    conf[g][d["color"]] += 1
        fn += len(gt) - len(matched)

        ctrl = BalloonController(cfg)
        ctrl.state = "APPROACH"
        out = ctrl.step(dets, w, h, now=0.05)
        summary = ", ".join(f"{d['label']}→{d['color'] or '?'}(b={d['bottom']:.2f} w={d['width']:.2f})"
                            for d in dets)
        print(f"{f['name']:22s} {out['state']:8s} v={out['v']:+4.0f} w={out['w']:+4.0f}  {summary}")
        if args.out:
            cv2.imwrite(os.path.join(args.out, f["name"] + ".jpg"), draw_car_overlay(img, out, cfg))

    print("\nColor HSV frente a la etiqueta (filas = verdad):")
    print("         " + " ".join(f"{p or 'dudoso':>7s}" for p in COLORS + (None,)))
    for g in COLORS:
        n = sum(conf[g].values())
        row = " ".join(f"{conf[g][p]:7d}" for p in COLORS + (None,))
        print(f"{g:8s} {row}   acierto={conf[g][g] / max(n, 1):.2f} (n={n})")

    if detector:
        detector.stop()
        prec = tp / max(tp + fp, 1)
        rec = tp / max(tp + fn, 1)
        print(f"\nTP={tp} FP={fp} FN={fn}  precisión={prec:.2f}  recall={rec:.2f}")


if __name__ == "__main__":
    main()

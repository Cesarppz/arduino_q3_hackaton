"""Evalúa el modelo YOLO (.eim) y la lógica del carro con un export de Edge Impulse.

    python eval_dataset.py <carpeta_export> --model model.eim [--out overlays/] [--color rojo]
    python eval_dataset.py <carpeta_export> --gt [--out overlays/]

- Con --model: corre el .eim sobre cada imagen y compara con las cajas
  etiquetadas de info.labels (acierto = misma etiqueta e IoU >= 0.3).
  El .eim tiene que ser de la arquitectura de la máquina (AARCH64 en el UNO Q,
  x86_64 en la laptop).
- Con --gt: no usa el modelo; convierte las cajas etiquetadas en una respuesta
  falsa del SDK para probar get_detections + BalloonController con imágenes reales.

--out guarda cada imagen con el overlay (cajas, estado, v/w).
"""
import argparse
import json
import os

import cv2

from car_core import BalloonController, CarConfig, draw_car_overlay, get_detections


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
    ap.add_argument("--gt", action="store_true")
    ap.add_argument("--out")
    ap.add_argument("--color", default="rojo")
    ap.add_argument("--min-conf", type=float, default=0.5)
    args = ap.parse_args()
    if not args.model and not args.gt:
        ap.error("usa --model <.eim> o --gt")

    info = json.load(open(os.path.join(args.export, "info.labels")))
    files = [f for f in info["files"] if f.get("boundingBoxes")]
    print(f"{len(files)} imágenes con cajas de {len(info['files'])}")

    runner = None
    if args.model:
        from fomo_core import load_model, run_inference
        runner, _ = load_model(args.model)
        model_w, model_h = runner.dim
    if args.out:
        os.makedirs(args.out, exist_ok=True)

    cfg = CarConfig(target_color=args.color, min_confidence=args.min_conf)
    tp = fp = fn = 0
    for f in files:
        img = cv2.imread(os.path.join(args.export, f["path"]))
        if img is None:
            continue
        h, w = img.shape[:2]
        gt = [(b["label"], (b["x"], b["y"], b["x"] + b["width"], b["y"] + b["height"]))
              for b in f["boundingBoxes"]]

        if runner:
            res = run_inference(runner, img)
            dets = get_detections(res, w, h, model_w, model_h, runner.resizeMode, cfg.min_confidence)
        else:
            res = {"result": {"bounding_boxes": [
                {"label": b["label"], "value": 1.0, "x": b["x"], "y": b["y"],
                 "width": b["width"], "height": b["height"]} for b in f["boundingBoxes"]]}}
            dets = get_detections(res, w, h, w, h, "squash", cfg.min_confidence)

        matched = set()
        for d in dets:
            box = (d["x1"], d["y1"], d["x2"], d["y2"])
            hit = next((i for i, (lab, g) in enumerate(gt)
                        if i not in matched and lab == d["label"] and iou(box, g) >= 0.3), None)
            if hit is None:
                fp += 1
            else:
                tp += 1
                matched.add(hit)
        fn += len(gt) - len(matched)

        ctrl = BalloonController(cfg)
        ctrl.state = "APPROACH"
        out = ctrl.step(dets, w, h, now=0.05)
        summary = ", ".join(f"{d['label']}(b={d['bottom']:.2f} w={d['width']:.2f})" for d in dets)
        print(f"{f['name']:22s} {out['state']:8s} v={out['v']:+4.0f} w={out['w']:+4.0f}  {summary}")
        if args.out:
            cv2.imwrite(os.path.join(args.out, f["name"] + ".jpg"), draw_car_overlay(img, out, cfg))

    if runner:
        runner.stop()
        prec = tp / max(tp + fp, 1)
        rec = tp / max(tp + fn, 1)
        print(f"\nTP={tp} FP={fp} FN={fn}  precisión={prec:.2f}  recall={rec:.2f}")


if __name__ == "__main__":
    main()

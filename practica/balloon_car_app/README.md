# Balloon Car (UNO Q)

Cámara → YOLO-Pro de Edge Impulse servido por el **brick Object Detection** (clases `azul` / `rojo` / `verde`) → `BalloonController` (PID sobre `dx`) → `drive(v, w)` vía Bridge → 2 motores DC.

## Modelo (brick Object Detection)
1. En App Lab, abre el brick **Object Detection** de la app → pestaña **AI models**. Con la cuenta de Edge Impulse vinculada aparece el proyecto `hackaton_arduinoq3` → **Download**.
2. **Brick Configuration** → elige ese modelo → **Save** (App Lab lo añade a `app.yaml`).
3. Run. En la consola debe salir `Modelo (brick): ... etiquetas=['azul', 'rojo', 'verde']`. Si salen etiquetas de COCO (`person`, ...), sigue cargado el modelo por defecto.

No hace falta `edge_impulse_linux` en el UNO Q. A cada frame se le hace resize al tamaño de entrada del modelo y se manda como JPEG al brick, así las cajas vuelven en coordenadas del modelo y `get_detections` las pasa al frame.

| Archivo | Qué hace |
|---|---|
| `python/main.py` | Loop de la app. **Arriba:** `TARGET_COLOR`, `WAIT_BUTTON`, `MOTOR_CAL` |
| `python/car_core.py` | Cajas YOLO → detecciones, PID, máquina de estados, overlay. `CarConfig` = parámetros |
| `python/vision.py` | `BrickDetector` (UNO Q), `EimDetector` (laptop, `.eim` x86_64) y `FrameSource` (cámara) |
| `python/fomo_core.py` | Conversión de coordenadas; carga de `.eim` local (solo laptop) |
| `python/calibrate.py` | `CAR_MODE=min_sweep / straight / spin` |
| `python/eval_dataset.py` | Prueba el modelo y la lógica con un export de Edge Impulse |
| `sketch/move.cpp`, `move.h` | Motores de la rama `valeria`, **sin cambios** |
| `sketch/sketch.ino` | `drive(v,w)` con compensación por motor + timeout + botón; registra también las funciones de `move.cpp` |

## Cómo se usa YOLO
- Las cajas tienen tamaño real. La cámara va baja y la parte de arriba del globo sale cortada, así que se usa el **borde inferior** de la caja (`bottom`) y su **ancho** (`width`) para saber si está cerca.
- El punto al que se apunta es el centro horizontal de la caja.
- En el dataset: lejos ≈ `bottom 0.3 / width 0.3`; a punto de tocar ≈ `bottom 0.6–0.7 / width 0.8–0.9`. Por eso `near_bottom=0.60`, `near_width=0.45`.
- El color sale de la etiqueta. Hay una confirmación HSV opcional (`use_hsv`), **apagada** porque con sol el verde sale casi blanco.

## Estados
`SEARCH` (gira a pulsos) → `APPROACH` (PID; esquiva otros colores) → `COMMIT` (embestida recta `t_commit`) → `BACKOFF` → `VERIFY` (¿sigue el globo? si no, +1) → `SEARCH` … `DONE` a los 3.

## Conexiones (de la rama `valeria`)
| Motor | Dirección | PWM |
|---|---|---|
| Izquierdo | D13 | D11 |
| Derecho | D12 | D3 |
| Botón arranque/paro (opcional, a GND) | D2 | |

## Puesta en marcha
1. Calibrar con las ruedas al aire: `CAR_MODE=min_sweep` → `MIN_IZQ / MIN_DER`. En el suelo: `CAR_MODE=straight` → `GAIN_*`. Copiar a `MOTOR_CAL` en `main.py`.
2. Si el carro gira al revés de lo esperado (globo a la derecha → gira a la izquierda), revisar el sentido en `move.cpp` o cambiar el signo de `w` en `drive()` del sketch.
3. `TARGET_COLOR` en `main.py` → Run.
4. Ajuste en pista (`CarConfig` en `car_core.py`): `kp/ki/kd`, `v_max`, `near_bottom`, `near_width`, `t_commit`.

## Probar
```bash
# Lógica con las cajas etiquetadas (laptop, sin modelo)
python eval_dataset.py ../../hackaton_arduinoq3-export --gt --color rojo --out overlays/
# Modelo real en el UNO Q (brick)
python eval_dataset.py <export> --brick --color rojo --out overlays/
# Modelo real en la laptop (build x86_64)
python eval_dataset.py <export> --model modelo_x86.eim --color rojo --out overlays/
```

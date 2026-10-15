# Balloon Car (UNO Q)

Cámara → YOLO-Pro de Edge Impulse servido por el **brick Object Detection** (clases `azul` / `rojo` / `verde`) → `BalloonController` (PID sobre `dx`) → `drive(v, w)` vía Bridge → 2 motores DC.

## Modelo (brick Object Detection) — Impulse #1 de `hackaton_arduinoq3`
El modelo **se carga desde Edge Impulse con el brick** en App Lab: *Brick Configuration → AI models* → proyecto `hackaton_arduinoq3`, Impulse #1. No hay que copiar ningún `.eim` a la placa y `app.yaml` no fija ninguna ruta.

Al arrancar, la consola debe mostrar `etiquetas=['azul', 'rojo', 'verde']`. Si el brick cargó otro modelo (p. ej. el YOLOX de COCO por defecto), la app **se detiene** con un error que lo dice.

Si reentrenas, vuelve a seleccionar el modelo en el brick para que App Lab descargue la versión nueva.

No hace falta `edge_impulse_linux` en el UNO Q. A cada frame se le hace resize al tamaño de entrada del modelo y se manda como JPEG al brick, así las cajas vuelven en coordenadas del modelo y `get_detections` las pasa al frame.

| Archivo | Qué hace |
|---|---|
| `python/main.py` | Loop de la app. **Arriba:** `TARGET_COLOR`, `WAIT_BUTTON`, `MOTOR_CAL` |
| `python/car_core.py` | Cajas YOLO → detecciones, PID, máquina de estados, overlay. `CarConfig` = parámetros |
| `python/vision.py` | `BrickDetector` (UNO Q), `EimDetector` (laptop, `.eim` x86_64) y `FrameSource` (cámara) |
| `python/tracker.py` | Modo híbrido: `BalloonTracker` (VitTrack) y `HybridPerception` (YOLO ↔ tracker) |
| `python/object_tracking_vittrack_2023sep.onnx` | Modelo del tracker (700 KB) |
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

## Modo híbrido YOLO + VitTrack (`USE_TRACKER = True` en `main.py`)
YOLO por el brick es lento, así que cuando encuentra el globo objetivo, VitTrack (OpenCV, unos ms por frame) lo sigue:
- **YOLO → tracker:** en `APPROACH`, con el objetivo recién detectado y todavía lejos.
- **Verificación:** cada `VERIFY_EVERY_S` (3 s; probar entre 2 y 4) se lanza YOLO en segundo plano sin parar el control. Se suelta el tracker si YOLO ve otro color en esa caja, si hay un globo de otro color en el camino o si no ve el globo 2 veces seguidas.
- **Tracker → YOLO** (en el mismo frame) también si baja el score del tracker (`track_min_score`), si el globo ya está **cerca** (la embestida y el aborto se deciden con YOLO) o si cambia el estado.
- Mientras se sigue con el tracker solo se ven los otros globos en cada verificación; cerca del objetivo se vuelve a YOLO en cada frame.
- La consola muestra `YOLO → tracker` / `tracker → YOLO (motivo)`. Si OpenCV no tiene `TrackerVit` (hace falta ≥ 4.9), avisa y sigue solo con YOLO.
- `TRACKER_MODEL` en `main.py` permite usar el `_int8bq.onnx` (más rápido; copiarlo a `python/`) si la OpenCV de la placa lo carga.

## Velocidades (+25 % tras la primera prueba en pista)
`v_max 75`, `v_min 31`, `v_commit 88`, `v_back 63`, `w_search 50`, `w_max 75`, `kp/ki/kd 88/19/10`, `k_repel 56`. Los tiempos (`t_commit`, `t_back`) no cambiaron, así que la embestida llega más lejos.

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

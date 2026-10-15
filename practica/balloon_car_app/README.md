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

## Control a 20 Hz (desacoplado de YOLO)
YOLO por el brick tarda **0.65–1.15 s por frame** y el sketch para los motores si pasan 300 ms sin comando. Con un comando por frame, el carro se movía 0.3 s y se paraba, así que la embestida casi no empujaba y los pulsos de búsqueda giraban poco.
- La cámara y YOLO van en un hilo (`PerceptionThread` en `main.py`). El control corre cada `CONTROL_DT` (50 ms) y manda un comando al menos cada 100 ms.
- Entre frames se llama a `ctrl.step(None, …)`. La embestida, el retroceso y los pulsos avanzan con el reloj, y APPROACH repite el último comando: gira solo `w_hold` (0.3 s) tras cada frame para no pasarse y mantiene `v`.
- SEARCH **gira y mira**: da un pulso de `search_on` a `w_search`, se para y espera un frame tomado ya quieto (sin que salga movido). Tras `search_pulses` pulsos sin ver nada pasa a WANDER.
- En cuanto ve un globo objetivo, va a por él en ese mismo frame (`confirm_frames=1`). Tras reventarlo, si en el frame de VERIFY ya se ve otro, va directo a por él.

| Parámetro (`CarConfig`) | Valor | Para qué |
|---|---|---|
| `v_commit` / `t_commit` | 100 / 1.2 s | embestida a tope |
| `near_bottom` / `near_width` / `commit_dx` | 0.50 / 0.38 / 0.25 | embiste antes, compensando la latencia |
| `v_max` / `v_min` / `align_dx` | 85 / 40 / 0.5 | se acerca más rápido y avanza aunque el globo esté de lado |
| `w_search` / `search_on` / `search_pulses` | 60 / 0.30 s / 16 | más ángulo por pulso y más vueltas antes de WANDER |
| `t_back` | 0.5 s | retroceso |
| `w_hold` / `stale_stop` | 0.3 s / 2 s | giro tras cada frame / parar si no llega percepción |

Si el carro **se pasa** al girar hacia el globo, baja `w_hold` o `kp`. Si en SEARCH gira **demasiado por pulso** y se salta globos, baja `search_on`.

## Modo híbrido YOLO + VitTrack (`USE_TRACKER`, apagado)
**No funciona en la placa:** su OpenCV 4.13 viene sin el módulo dnn que usa VitTrack (`the tracking module needs to be built with opencv_dnn`), y no se puede instalar otra porque la versión está fijada (por eso `requirements.txt` ya no pide OpenCV). Sigue disponible en la laptop.
- **YOLO → tracker:** en `APPROACH`, con el objetivo recién detectado y todavía lejos.
- **Verificación:** cada `VERIFY_EVERY_S` se lanza YOLO en segundo plano. Se suelta el tracker si YOLO ve otro color en esa caja, si hay un globo de otro color en el camino o si no ve el globo 2 veces seguidas.
- **Tracker → YOLO** también si baja el score, si el globo ya está cerca o si cambia el estado.

## Estados
`SEARCH` (gira y mira) → `APPROACH` (PID; esquiva otros colores) → `COMMIT` (embestida a 100 durante `t_commit`) → `BACKOFF` → `VERIFY` (con un frame tomado ya parado: ¿sigue el globo? reintento; si no, +1 y a por el siguiente) → `SEARCH` … `DONE` a los 3.

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
4. Ajuste en pista (`CarConfig` en `car_core.py`): `kp/ki/kd`, `w_hold`, `v_max`, `near_bottom`, `near_width`, `t_commit`, `search_on`.

## Probar
```bash
# Lógica con las cajas etiquetadas (laptop, sin modelo)
python eval_dataset.py ../../hackaton_arduinoq3-export --gt --color rojo --out overlays/
# Modelo real en el UNO Q (brick)
python eval_dataset.py <export> --brick --color rojo --out overlays/
# Modelo real en la laptop (build x86_64)
python eval_dataset.py <export> --model modelo_x86.eim --color rojo --out overlays/
```

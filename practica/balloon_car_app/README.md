# Balloon Car (UNO Q)

Cámara → YOLO-Pro de Edge Impulse servido por el **brick Object Detection** (una clase: `globo`) → **color por HSV** dentro de cada caja → `BalloonController` (gira y mira) → `drive(v, w)` vía Bridge → 2 motores DC.

## Modelo (brick Object Detection): una sola clase `globo`
El modelo **se carga desde Edge Impulse con el brick** en App Lab (*Brick Configuration → AI models*). No hay que copiar ningún `.eim` a la placa y `app.yaml` no fija ninguna ruta.

Al arrancar, la consola debe mostrar `Etiquetas ['globo'] → color por HSV`. Cualquier modelo de **una sola etiqueta** vale. También se acepta el modelo antiguo de 3 colores (Impulse #1), pero se ignora su etiqueta y el color sale igualmente de HSV. Si el brick cargó otro modelo (p. ej. el YOLOX de COCO por defecto), la app **se detiene** con un error que lo dice.

Si reentrenas, vuelve a seleccionar el modelo en el brick para que App Lab descargue la versión nueva.

No hace falta `edge_impulse_linux` en el UNO Q. A cada frame se le hace resize al tamaño de entrada del modelo y se manda como JPEG al brick, así las cajas vuelven en coordenadas del modelo y `get_detections` las pasa al frame.

| Archivo | Qué hace |
|---|---|
| `python/main.py` | Loop de la app. **Arriba:** `TARGET_COLOR`, `WAIT_BUTTON`, `MOTOR_CAL` |
| `python/car_core.py` | Cajas YOLO → detecciones, color HSV (`color_probs`), máquina de estados, overlay. `CarConfig` = parámetros |
| `python/vision.py` | `BrickDetector` (UNO Q), `EimDetector` (laptop, `.eim` x86_64) y `FrameSource` (cámara) |
| `python/tracker.py` | Modo híbrido: `BalloonTracker` (VitTrack) y `HybridPerception` (YOLO ↔ tracker) |
| `python/object_tracking_vittrack_2023sep.onnx` | Modelo del tracker (700 KB) |
| `python/fomo_core.py` | Conversión de coordenadas; carga de `.eim` local (solo laptop) |
| `python/calibrate.py` | `CAR_MODE=min_sweep / straight / spin` |
| `python/eval_dataset.py` | Prueba el modelo, el color HSV (matriz de confusión) y la lógica con un export de Edge Impulse |
| `sketch/move.cpp`, `move.h` | Motores de la rama `valeria`, **sin cambios** |
| `sketch/sketch.ino` | `drive(v,w)` con compensación por motor + timeout + botón; registra también las funciones de `move.cpp` |

## Cómo se usa YOLO
- Las cajas tienen tamaño real. La cámara va baja y la parte de arriba del globo sale cortada, así que se usa el **borde inferior** de la caja (`bottom`) y su **ancho** (`width`) para saber si está cerca.
- El punto al que se apunta es el centro horizontal de la caja.
- En el dataset: lejos ≈ `bottom 0.3 / width 0.3`; a punto de tocar ≈ `bottom 0.6–0.7 / width 0.8–0.9`. Con la latencia de YOLO se embiste algo antes: `near_bottom=0.50`, `near_width=0.38`.

## Color por HSV
El modelo solo dice "globo", así que el color se decide en `classify_detections` (`car_core.py`):
1. Se recorta la zona central de la caja, sin los bordes ni el 30 % inferior (la base negra).
2. Solo votan los píxeles **con color** (`S ≥ hsv_s_min` 25 y `V ≥ hsv_v_min` 40). Cada uno vota por su tono según `hsv_hues`: rojo 0–10 y 165–179, verde 35–90, azul 95–135.
3. La probabilidad de cada color es su parte de los votos. Gana el más votado si tiene `≥ hsv_min_prob` (0.65) y si al menos `hsv_min_colored` (5 %) del recorte tiene color.
4. Si no, el globo queda **dudoso** (`color=None`) y se trata como obstáculo: se esquiva, nunca se embiste.

Como la probabilidad se calcula solo entre los píxeles con color, un verde lavado por el sol sigue saliendo verde.

Con el export de Downloads (402 cajas con color, `eval_dataset.py --gt`): **96 % de aciertos, 1.5 % de color equivocado y 2.5 % dudosos**. Además, el carro solo embiste tras ver el color objetivo en 2 frames seguidos (`commit_confirm`). El overlay y el log muestran `rojo 0.82` (color y probabilidad).

Para ajustar con fotos de la pista: súbelas al proyecto viejo de 3 colores (o etiquétalas por color), exporta y corre `python eval_dataset.py <export>/training --gt`. Al final sale la matriz de confusión.

## Control a 20 Hz + "gira y mira"
YOLO por el brick tarda **0.65–1.4 s por frame**, y el sketch para los motores si pasan 300 ms sin comando.
- La cámara y YOLO van en un hilo (`PerceptionThread` en `main.py`). El control corre cada `CONTROL_DT` (50 ms) y manda un comando al menos cada 100 ms. Entre frames se llama a `ctrl.step(None, …)`, y la embestida, el retroceso y los giros avanzan con el reloj.
- **Solo deciden los frames tomados con el carro quieto**, es decir, al menos `look_settle` (0.15 s) después del último giro. Antes, el carro decidía con un frame tomado mientras giraba: cuando YOLO respondía, el globo ya no estaba donde decía la caja, el carro giraba de más y lo perdía.
- **SEARCH:** gira un pulso (`search_on` a `w_search`), para y espera un frame quieto. Si ve el globo en un frame tomado mientras giraba, corta el pulso y lo confirma con un frame quieto.
- **APPROACH**, con cada frame quieto:
  - `|dx| > align_dx`: hace un giro **corto** en sitio, de `turn_gain·|dx|` s (entre 0.06 y 0.30 s) a `w_align`. Luego para y vuelve a mirar. Nunca encadena dos giros sin mirar.
  - Centrado y lejos: avanza recto a `v_approach` con una corrección suave (`k_fine`).
  - Centrado y cerca: para. **Embiste solo tras `commit_confirm` (2) frames quietos seguidos** en los que el globo está centrado, cerca, es del color objetivo y no hay otro globo delante.
- **COMMIT:** embiste recto a 100 durante 1.2 s, sin corregir, porque los frames llegan tarde.
- Tras reventar un globo, si en el frame de VERIFY ya se ve otro, va directo a por él.
- Cada decisión se imprime en la consola, por ejemplo `rojo 0.97 dx=+0.41 b=0.30 w=0.20 → girar 0.14 s` o `... cerca → apuntado 1/2`.

| Parámetro (`CarConfig`) | Valor | Para qué |
|---|---|---|
| `align_dx` / `w_align` / `turn_gain` | 0.20 / 45 / 0.35 s | giro corto para centrar el globo |
| `v_approach` / `k_fine` | 55 / 30 | avance con el globo centrado |
| `near_bottom` / `near_width` / `commit_dx` | 0.50 / 0.38 / 0.15 | cuándo está cerca y centrado para embestir |
| `commit_confirm` | 2 | frames quietos apuntado antes de embestir (≈1–1.5 s) |
| `v_commit` / `t_commit` / `t_back` | 100 / 1.2 s / 0.5 s | embestida y retroceso |
| `w_search` / `search_on` / `search_pulses` | 60 / 0.30 s / 16 | búsqueda |
| `look_settle` / `stale_stop` | 0.15 s / 2 s | frame "quieto" / parar si no llega percepción |

**Calibrar `turn_gain`** con `CAR_MODE=spin`: mide cuántos grados gira en 2 s a `w_align` (45). La cámara C270 tiene unos 60° de campo, así que `dx=1` equivale a unos 30°: `turn_gain ≈ 30 / (grados por segundo)`.
- Si se pasa (el `dx` cambia de signo en el giro siguiente), baja `turn_gain`.
- Si se queda corto (necesita muchos giros), súbelo.

## Modo híbrido YOLO + VitTrack (`USE_TRACKER`, apagado)
**No funciona en la placa:** su OpenCV 4.13 viene sin el módulo dnn que usa VitTrack (`the tracking module needs to be built with opencv_dnn`), y no se puede instalar otra porque la versión está fijada (por eso `requirements.txt` ya no pide OpenCV). Sigue disponible en la laptop.
- **YOLO → tracker:** en `APPROACH`, con el objetivo recién detectado y todavía lejos.
- **Verificación:** cada `VERIFY_EVERY_S` se lanza YOLO en segundo plano. Se suelta el tracker si YOLO ve otro color en esa caja, si hay un globo de otro color en el camino o si no ve el globo 2 veces seguidas.
- **Tracker → YOLO** también si baja el score, si el globo ya está cerca o si cambia el estado.

## Estados
`SEARCH` (gira y mira) → `APPROACH` (gira corto / avanza / apunta; esquiva otros colores y los dudosos) → `COMMIT` (embestida a 100 durante `t_commit`) → `BACKOFF` → `VERIFY` (con un frame tomado ya parado: ¿sigue el globo? reintento; si no, +1 y a por el siguiente) → `SEARCH` … `DONE` a los 3.

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
4. Ajuste en pista (`CarConfig` en `car_core.py`): `turn_gain`, `v_approach`, `near_bottom`, `near_width`, `commit_confirm`, `t_commit`, `search_on`; color: `hsv_s_min`, `hsv_min_prob`.

## Probar
```bash
# Lógica con las cajas etiquetadas (laptop, sin modelo)
python eval_dataset.py ../../hackaton_arduinoq3-export --gt --color rojo --out overlays/
# Modelo real en el UNO Q (brick)
python eval_dataset.py <export> --brick --color rojo --out overlays/
# Modelo real en la laptop (build x86_64)
python eval_dataset.py <export> --model modelo_x86.eim --color rojo --out overlays/
```

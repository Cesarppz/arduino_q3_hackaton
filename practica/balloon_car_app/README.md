# Balloon Car (UNO Q)

Cámara → FOMO (`.eim`) → verificación HSV del color → `BalloonController` (PID sobre `dx`) → `drive(v, w)` vía Bridge → 2 servos continuos.

| Archivo | Qué hace |
|---|---|
| `python/fomo_core.py` | Copia de `uno_q_app/python/fomo_core.py` (inferencia, centroides, filtro) |
| `python/car_core.py` | Color HSV, PID, máquina de estados, transportes `drive`, overlay |
| `python/main.py` | Loop de la app; `SERVO_CAL` = calibración de servos |
| `python/calibrate.py` | Modos `CAR_MODE=stop_sweep / straight / spin` |
| `sketch/sketch.ino` | `drive`, `raw_us`, `set_cal`, `set_running`, `get_color`; botón y timeout |

## Estados
`SEARCH` (gira a pulsos) → `APPROACH` (PID; esquiva otros colores) → `COMMIT` (embestida recta `t_commit`) → `BACKOFF` → `VERIFY` (¿sigue el globo? si no, +1) → `SEARCH` … `DONE` a los 3.

## Conexiones
| Señal | Pin |
|---|---|
| Servo izquierdo | D9 |
| Servo derecho (espejado, `SIGN = -1`) | D10 |
| Botón a GND | D2 |
Servos alimentados con fuente de 5–6 V aparte, **GND común** con el UNO Q.

## Puesta en marcha
1. Modelo AARCH64 → `python/model.eim`.
2. Calibrar (ruedas al aire): `CAR_MODE=stop_sweep` → `STOP_L/STOP_R`; en el suelo `CAR_MODE=straight` → `K_*`. Copiar a `SERVO_CAL` en `main.py`.
3. Color: `TARGET_COLOR=rojo|verde|azul`, o con el botón (corta = color, LED parpadea 1/2/3; larga = arrancar / parar).
4. Ajuste en pista (`CarConfig` en `car_core.py`): `kp/ki/kd`, `v_max`, `near_y`, `near_area`, `t_commit`.

## Probar en la laptop
```bash
cd practica/balloon_car_app/python
SHOW=1 TARGET_COLOR=rojo EI_MODEL=<modelo_x86_64>.eim ../../../.venv/bin/python main.py
```
| Síntoma | Ajuste |
|---|---|
| Oscila al apuntar | bajar `kp`, subir `kd` |
| Se va de lado aunque centra | subir `ki` (o mejorar `K_*`) |
| Embiste demasiado pronto/tarde | `near_y` / `near_area` |
| No llega a reventar | subir `t_commit` / `v_commit` |
| Gira al revés | invertir `SIGN` en el sketch |

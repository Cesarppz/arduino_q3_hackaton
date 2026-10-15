"""Rutinas de calibración de los servos (se lanzan con CAR_MODE=<modo> en main.py).

Cada servo de rotación continua tiene su propio punto de parada y su propia
"fuerza": sin calibrar, el carro no va recto. El PID con cámara corrige el
resto, pero cuanto mejor calibrado, más rápido y recto llega al globo.

Modos (no interactivos: mira las ruedas y la consola):
    stop_sweep  barre µs de 1400 a 1600 en cada servo (1 s por valor). Anota
                el centro del rango donde la rueda está quieta → STOP_L / STOP_R.
    straight    avanza 2 s a v=60, para 3 s, repite. Si se desvía a la derecha,
                sube K_R_FWD (o baja K_L_FWD) en SERVO_CAL de main.py.
                Repetir con v=-60 para los K_*_REV.
    spin        gira en sitio 2 s a la derecha y 2 s a la izquierda; ambos
                giros deberían ser parecidos (útil para elegir w_search y kp).
"""
import time

SWEEP = list(range(1400, 1601, 10))


def make_loop(mode, transport, sender):
    t0 = time.monotonic()

    if mode == "stop_sweep":
        state = {"i": -1}

        def loop():
            i = int((time.monotonic() - t0) // 1.0)
            n = len(SWEEP)
            if i >= 2 * n:
                transport.raw_us(0, 0)  # 0 = soltar (sin pulso)
                time.sleep(0.2)
                return
            us = SWEEP[i % n]
            if i != state["i"]:
                state["i"] = i
                print(f"{'IZQUIERDO' if i < n else 'DERECHO':10s} {us} µs")
            # se reenvía cada 50 ms: el sketch para si pasan 300 ms sin mensajes
            transport.raw_us(us, 0) if i < n else transport.raw_us(0, us)
            time.sleep(0.05)
        return loop

    if mode in ("straight", "spin"):
        def loop():
            ph = (time.monotonic() - t0) % 10.0
            if mode == "straight":
                v, w = (60, 0) if ph < 2 else (0, 0) if ph < 5 else (-60, 0) if ph < 7 else (0, 0)
            else:
                v, w = (0, 50) if ph < 2 else (0, 0) if ph < 5 else (0, -50) if ph < 7 else (0, 0)
            if sender.send(v, w) and (v or w):
                print(f"v={v:+d} w={w:+d}")
            time.sleep(0.05)
        return loop

    raise ValueError(f"CAR_MODE desconocido: {mode}")

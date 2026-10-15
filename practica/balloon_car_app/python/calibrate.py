"""Rutinas de calibración de los motores DC (se lanzan con CAR_MODE=<modo> en main.py).

Cada motor tiene su propio impulso y su propia zona muerta: sin compensar, el
carro no va recto. La cámara corrige el resto, pero cuanto mejor
compensado, más rápido y recto llega al globo.

Modos (no interactivos: mira las ruedas y la consola):
    min_sweep  sube la velocidad de cada motor de 0 a 50 % (0.5 s por paso).
               Anota el % en que la rueda EMPIEZA a girar → MIN_IZQ / MIN_DER.
    straight   avanza 2 s a v=60, para 3 s, retrocede 2 s, para 3 s.
               Si se desvía a la derecha, el izquierdo va más rápido: baja
               GAIN_IZQ (o sube GAIN_DER) en MOTOR_CAL de main.py.
    spin       gira en sitio 2 s a cada lado; ambos giros deberían ser parecidos
               (útil para elegir w_search y turn_gain). Para turn_gain: grados
               girados en 2 s → °/s a esa w; con la cámara de ~60° de campo,
               dx=1 son ~30°, así que turn_gain ≈ 30 / (°/s a w_align).
               Para el tiempo de 360° está test_calibracion.py de la rama valeria.
"""
import time

SWEEP = list(range(0, 51, 2))
STEP_S = 0.5


def make_loop(mode, transport, sender):
    t0 = time.monotonic()

    if mode == "min_sweep":
        state = {"i": -1}

        def loop():
            i = int((time.monotonic() - t0) // STEP_S)
            n = len(SWEEP)
            if i >= 2 * n:
                time.sleep(0.2)  # sin mensajes → el sketch para los motores
                return
            pct = SWEEP[i % n]
            if i != state["i"]:
                state["i"] = i
                print(f"{'IZQUIERDO' if i < n else 'DERECHO':10s} {pct:3d} %")
            # se reenvía cada 50 ms: el sketch para si pasan 300 ms sin mensajes
            transport.raw_pct(pct, 0) if i < n else transport.raw_pct(0, pct)
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

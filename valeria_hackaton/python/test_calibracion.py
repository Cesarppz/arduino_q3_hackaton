import time

from arduino.app_utils.bridge import Bridge

# --- AJUSTA ESTE VALOR HASTA QUE HAGA EXACTAMENTE 1 VUELTA COMPLETA ---
VELOCIDAD_GIRO = 60
TIEMPO_360_GRADOS = 4.1# Segundos estimados (ajustar en la pista)
# --------------------------------------------------------------------

def test_giro_360():
    
    print("Iniciando calibración de 360 grados...")
    time.sleep(2) 
    
    # Arranca el giro
    Bridge.call("MotorIzquierdo", VELOCIDAD_GIRO)
    
    # Mantiene el giro por el tiempo definido
    time.sleep(TIEMPO_360_GRADOS)
    
    # Freno en seco
    Bridge.call("PararMotores")
    
    # Cálculos automáticos
    print(f"Giro completado en {TIEMPO_360_GRADOS} segundos.")
    print(f"-> Tu tiempo para 90 grados será: {TIEMPO_360_GRADOS / 4} segundos.")
    print(f"-> Tu tiempo para 180 grados será: {TIEMPO_360_GRADOS / 2} segundos.")


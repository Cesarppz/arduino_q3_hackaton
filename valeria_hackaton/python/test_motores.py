import time
from arduino.app_utils import App
from arduino.app_utils.bridge import Bridge

def test_todas_las_funciones():
    # Pausa inicial obligatoria: da tiempo a que el hardware y el Bridge en C++ arranquen
    time.sleep(2) 
    print("Iniciando batería de pruebas de hardware...")
    
    # ---------------------------------------------------------
    # PRUEBA 1: Función directa 'Go' (Ambos motores adelante)
    # ---------------------------------------------------------
    print("Prueba 1: Avanzar (Llamada directa a Go)")
    Bridge.call("Go", 20)              # Enviamos un número entero (int)
    time.sleep(2)                      # Dejamos que el robot avance 2 segundos
    Bridge.call("PararMotores")     # Freno de seguridad
    time.sleep(2)                      # Pausa de 2 segundos con los motores apagados

    # ---------------------------------------------------------
    # PRUEBA 2: Función directa 'MotorIzquierdo'
    # ---------------------------------------------------------
    print("Prueba 2: Motor Izquierdo Individual")
    Bridge.call("MotorIzquierdo", 20)  # Solo debería girar la rueda izquierda
    time.sleep(2)                      
    Bridge.call("PararMotores")     
    time.sleep(2)                      

    # ---------------------------------------------------------
    # PRUEBA 3: Función directa 'MotorDerecho'
    # ---------------------------------------------------------
    print("Prueba 3: Motor Derecho Individual")
    Bridge.call("MotorDerecho", 20)    # Solo debería girar la rueda derecha
    time.sleep(2)                      
    Bridge.call("PararMotores")     
    time.sleep(2)                      

    # ---------------------------------------------------------
    # PRUEBA 4: Función procesadora de texto 'GoMotores' (Comando GO)
    # ---------------------------------------------------------
    print("Prueba 4: Avanzar usando el parser de texto (GO 20)")
    Bridge.call("GoMotores", "GO 20")  # Enviamos un String que C++ va a trocear
    time.sleep(2)                      
    Bridge.call("PararMotores")     
    time.sleep(2)                      

    # ---------------------------------------------------------
    # PRUEBA 5: Función procesadora de texto 'GoMotores' (Comando LEFT)
    # ---------------------------------------------------------
    print("Prueba 5: Girar usando el parser de texto (LEFT 60)")
    Bridge.call("GoMotores", "LEFT 20") # El C++ debe invertir un motor y avanzar el otro
    time.sleep(2)                       
    Bridge.call("PararMotores")      
    time.sleep(2)                       

    print("Batería de pruebas finalizada con éxito. El robot debe estar quieto.")

# Ejecutamos la secuencia al iniciar el script
test_todas_las_funciones()

# Mantenemos viva la aplicación para que el Bridge cierre correctamente
App.run()
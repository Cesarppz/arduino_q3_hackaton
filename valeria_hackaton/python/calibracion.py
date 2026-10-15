import time
from arduino.app_utils.bridge import Bridge

VELOCIDAD_GIRO = 60
TIEMPO_90 = 1.025
TIEMPO_180= 2.05

def giro_90_izq():
    Bridge.call("MotorDerecho", VELOCIDAD_GIRO)
    time.sleep(TIEMPO_90)
    Bridge.call("PararMotores")
    
def giro_180_izq() :
    Bridge.call("MotorDerecho", VELOCIDAD_GIRO)
    time.sleep(TIEMPO_180)
    Bridge.call("PararMotores")
    

def giro_90_der() :
    Bridge.call("MotorIzquierdo", VELOCIDAD_GIRO)
    time.sleep(TIEMPO_90)
    Bridge.call("PararMotores")

def giro_180_der() :
    Bridge.call("MotorIzquierdo", VELOCIDAD_GIRO)
    time.sleep(TIEMPO_180)
    Bridge.call("PararMotores")




    
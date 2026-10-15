#include <Arduino.h>
#include <Arduino_RouterBridge.h> 
#include "move.h"              

void setup() {
  Serial.begin(115200);


  ConfigurarMotores();

  Bridge.provide_safe("GoMotores", GoMotores);
  Bridge.provide_safe("MotorIzquierdo", MotorIzquierdo);
  Bridge.provide_safe("MotorDerecho", MotorDerecho);
  Bridge.provide_safe("Go", Go);
  Bridge.provide_safe("PararMotores", PararMotores);
  Bridge.provide_safe("GoMotores", GoMotores);
}

void loop() {
  Bridge.update();
  delay(10);
}
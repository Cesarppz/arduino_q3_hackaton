#ifndef PULSADORES_H
#define PULSADORES_H

#include <Arduino.h>

class RobotControl {
  public:
    void inicializar();
    int verificarPulsadores();
    void encender(int led);
    void apagar(int led);
};

#endif
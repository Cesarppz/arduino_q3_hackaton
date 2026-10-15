#ifndef MOVE_H
#define MOVE_H

#include <Arduino.h>


extern const int motorIzqPin;
extern const int motorIzqPWM;
extern const int motorDerPin;
extern const int motorDerPWM;

extern int pinsIzq[]; 
extern int pinsDer[]; 

int is_speed(String parametroRecibido);
void MotorIzquierdo(int velocidad);
void MotorDerecho(int velocidad);
void Go(int velocidad);
void PararMotores();
void GoMotores(String comando);
void ConfigurarMotores();


#endif
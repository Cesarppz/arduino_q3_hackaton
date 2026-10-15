#include <Arduino.h>
#include <Arduino_RouterBridge.h>


const int motorIzqPin = 13;
const int motorIzqPWM = 11;
const int motorDerPin = 12;
const int motorDerPWM = 3;


int pinsIzq[] = {motorIzqPin, motorIzqPWM}; 
int pinsDer[] = {motorDerPin, motorDerPWM}; 


int is_speed(String parametroRecibido) {
    int velocidad = parametroRecibido.toInt(); 

    if (velocidad == 0 && parametroRecibido != "0") {
        Serial.println("Error: speed not valid"); 
        return -1; // Retorna error
    }
    else {
        velocidad = constrain(velocidad, -100, 100);
        Serial.println("Speed received from Bridge: " + String(velocidad));
        return velocidad;
    }
}

void MotorIzquierdo(int velocidad) {
  int pwm = map(abs(velocidad), 0, 100, 0, 255);

  if (velocidad >= 0) {
    digitalWrite(pinsIzq[0], HIGH); 
  } else {
    digitalWrite(pinsIzq[0], LOW);
  }

  analogWrite(pinsIzq[1], pwm); // El índice 1 es el PWM
}

void MotorDerecho(int velocidad) {
  int pwm = map(abs(velocidad), 0, 100, 0, 255);

  if (velocidad >= 0) {

    digitalWrite(pinsDer[0], LOW); 
  } else {
    digitalWrite(pinsDer[0], HIGH);
  }

  analogWrite(pinsDer[1], pwm);
}

void Go (int velocidad) {

  MotorDerecho(velocidad);
  MotorIzquierdo(velocidad); 
}

void PararMotores() {
  MotorIzquierdo(0);
  MotorDerecho(0);
}


void GoBack(int velocidad) {
  Go(-velocidad);

}


void GoMotores(String comando) {

  int espacio = comando.indexOf(' ');
  
  if (espacio == -1) {
    Serial.println("ERROR: Formato incorrecto. Usa 'GO 50' o 'LEFT 50'");
    PararMotores(); // Medida de seguridad si Python manda algo raro
    return;
  }

  // Separamos el string en dos partes usando el espacio como corte
  String accion = comando.substring(0, espacio);       // Ejemplo: "LEFT"
  String valorTexto = comando.substring(espacio + 1);  // Ejemplo: "50"

  // Llamamos a is_speed solo pasándole el texto del número
  int velocidad = is_speed(valorTexto);

  if (velocidad == -1) return; // Si la validación falló, salimos

  // NOTA 9: equalsIgnoreCase se usa pasándole el texto entre los paréntesis.
  if (accion.equalsIgnoreCase("LEFT")) {
     // Gira a la izquierda
     MotorIzquierdo(-velocidad);
     MotorDerecho(velocidad);
  }
  else if (accion.equalsIgnoreCase("GO")) {
     Go(velocidad);
  }
  else if (accion.equalsIgnoreCase("STOP")) {
     PararMotores();
  }
  else if (accion.equalsIgnoreCase("BACK")) {
   GoBack(velocidad);
  }
}

void ConfigurarMotores() {
  pinMode(pinsIzq[0], OUTPUT);
  pinMode(pinsIzq[1], OUTPUT);
  pinMode(pinsDer[0], OUTPUT);
  pinMode(pinsDer[1], OUTPUT);

  PararMotores(); 
}
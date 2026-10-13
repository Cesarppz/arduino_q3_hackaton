#include <Stepper.h>
int stepsPerRevolution = 2048;
int speed = 10;
int pinLed = 13;
Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

void setup() {
  myStepper.setSpeed(speed);
  pinMode(LED_BUILTIN, OUTPUT);

}

void loop() {
  myStepper.step(stepsPerRevolution);

  digitalWrite(LED_BUILTIN, HIGH);   // Enciende el LED (manda 5V)
  delay(1000);                       // Espera 1 segundo (1000 milisegundos)
  digitalWrite(LED_BUILTIN, LOW);    // Apaga el LED (manda 0V)
  delay(1000);

}

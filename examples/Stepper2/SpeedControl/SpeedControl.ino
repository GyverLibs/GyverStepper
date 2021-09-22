// крутим мотор. Отправляй в сериал целое число, шаг/сек

#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 3);

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(10);
}

void loop() {
  // здесь происходит движение моторов, вызывать как можно чаще
  stepper.tick();

  // управляем скоростью
  if (Serial.available() > 0) {
    int val = Serial.parseInt();
    stepper.setSpeed(val);
  }

  static uint32_t tmr;
  if (millis() - tmr >= 30) {
    tmr = millis();
    Serial.println(stepper.pos);
  }
}

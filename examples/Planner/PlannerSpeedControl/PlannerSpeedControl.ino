// крутим мотор. Отправляй в сериал целое число, шаг/сек

#include "GyverPlanner.h"
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);
GPlanner<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(57600);
  // добавляем шаговики на оси
  planner.addStepper(0, stepper1);  // ось 0
  planner.addStepper(1, stepper2);  // ось 1

  Serial.setTimeout(10);
}

void loop() {
  // здесь происходит движение моторов, вызывать как можно чаще
  planner.tick();

  // управляем скоростью
  if (Serial.available() > 0) {
    int val = Serial.parseInt();
    planner.setSpeed(0, val);
  }
}

// крутимся с заданной скоростью

#include <GyverStepper.h>
//GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);

void setup() {
  stepper.setRunMode(KEEP_SPEED); // режим поддержания скорости
  stepper.setSpeedDeg(50);        // в градусах/сек
}

void loop() {
  stepper.tick();
}

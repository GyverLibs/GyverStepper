// управляем скоростью из СОМ порта
// отправь q для тормоза
// отправь w для плавной остановки
// отправь e для скорости 5 град/сек
// отправь r для скорости 100 град/сек

#include <GyverStepper.h>
GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);

void setup() {
  Serial.begin(9600);
  stepper.setRunMode(KEEP_SPEED); // режим поддержания скорости
  stepper.setSpeedDeg(5);         // в градусах/сек
}

void loop() {
  stepper.tick();
  if (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'q') stepper.brake();
    if (ch == 'w') stepper.stop();
    if (ch == 'e') stepper.setSpeedDeg(5);
    if (ch == 'r') stepper.setSpeedDeg(100);
  }
}

// крутим туда сюда, тикаем в loop

#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 3);

void setup() {
  Serial.begin(9600);
  //stepper.enable();
  stepper.setMaxSpeed(100);     // скорость движения к цели
  stepper.setAcceleration(200); // ускорение
  stepper.setTarget(300);       // цель
}

bool dir = 1;
void loop() {
  stepper.tick();   // мотор асинхронно крутится тут

  // если приехали
  if (stepper.ready()) {
    dir = !dir;   // разворачиваем
    stepper.setTarget(dir * 300); // едем в другую сторону
  }

  // асинхронный вывод в порт
  static uint32_t tmr;
  if (millis() - tmr >= 30) {
    tmr = millis();
    Serial.println(stepper.pos);
  }
}

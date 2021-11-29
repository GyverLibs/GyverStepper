// крутим туда сюда, тикаем в loop

// включаем быстрый профиль, 10 участков
#define GS_FAST_PROFILE 10
#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 5);

uint32_t tar = 60000;
bool dir = 1;

void setup() {
  Serial.begin(9600);

  //stepper.enable();
  stepper.setMaxSpeed(30000);     // скорость движения к цели
  stepper.setAcceleration(30000); // ускорение

  stepper.setTarget(tar);       // цель
}

void loop() {
  while (1) {
    stepper.tick();   // мотор асинхронно крутится тут

    // если приехали
    if (stepper.ready()) {
      dir = !dir;   // разворачиваем
      stepper.setTarget(dir * tar); // едем в другую сторону
    }

    // асинхронный вывод в порт
    static uint32_t tmr;
    if (millis() - tmr >= 30) {
      tmr = millis();
      Serial.println(stepper.pos);
    }
  }
}

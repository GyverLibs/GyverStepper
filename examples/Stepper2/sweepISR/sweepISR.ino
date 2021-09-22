// крутим туда сюда, прерывание таймера

#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 3);

void setup() {
  Serial.begin(9600);
  initTimer();
  //stepper.enable();
  stepper.setMaxSpeed(100);     // скорость движения к цели
  stepper.setAcceleration(200); // ускорение
  stepper.setTarget(300);       // цель
  setPeriod(stepper.getPeriod());
  startTimer();
}

// прерывание таймера
ISR(TIMER1_COMPA_vect) {
  // здесь происходит движение мотора
  // если мотор должен двигаться (true) - ставим новый период таймеру
  if (stepper.tickManual()) setPeriod(stepper.getPeriod());
  else stopTimer();
  // если нет - останавливаем таймер
}

bool dir = 1;
void loop() {
  // если приехали
  if (stepper.ready()) {
    dir = !dir;   // разворачиваем
    stepper.setTarget(dir * 300); // едем в другую сторону
    setPeriod(stepper.getPeriod());
    startTimer();
  }

  // асинхронный вывод в порт
  static uint32_t tmr;
  if (millis() - tmr >= 30) {
    tmr = millis();
    Serial.println(stepper.pos);
  }
}

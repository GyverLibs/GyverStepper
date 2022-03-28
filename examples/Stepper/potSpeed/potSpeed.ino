// установка скорости потенциометром

#include <GyverStepper.h>
//GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);

void setup() {
  stepper.setRunMode(KEEP_SPEED); // режим поддержания скорости
  stepper.setSpeedDeg(50);        // в градусах/сек
}

void loop() {
  stepper.tick();

  // сделаем таймер на 50 мс и будем опрашивать потенциометр
  // менять скорость чаще нет смысла
  static uint32_t tmr2;
  if (millis() - tmr2 > 50) {
    tmr2 = millis();

    // ставим новую скорость (-512.. 512 шагов в секунду)
    // будет крутиться в разные стороны
    stepper.setSpeed(512 - analogRead(0));
  }
}

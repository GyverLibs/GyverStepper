// пример калибровки нуля по концевикам
// концевик на D6
#define LIMSW_X 6

#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 3);

void setup() {
  // пуллапим. Кнопки замыкают на GND
  pinMode(LIMSW_X, INPUT_PULLUP);
}

void loop() {
}

void homing() {
  if (digitalRead(LIMSW_X)) {       // если концевик X не нажат
    stepper.setSpeed(-10);       // ось Х, -10 шаг/сек
    while (digitalRead(LIMSW_X)) {  // пока кнопка не нажата
      stepper.tick();               // крутим
    }
    // кнопка нажалась - покидаем цикл
    stepper.brake();                // тормозим, приехали
  }
  stepper.reset();    // сбрасываем координаты в 0
}

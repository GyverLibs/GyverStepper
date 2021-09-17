// пример калибровки нуля по концевикам
// концевики на D6 и D7
#define LIMSW_X 6
#define LIMSW_Y 7

#include "GyverPlanner.h"
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);
GPlanner<STEPPER2WIRE, 2> planner;

void setup() {
  // добавляем шаговики на оси
  planner.addStepper(0, stepper1);  // ось 0
  planner.addStepper(1, stepper2);  // ось 1

  // устанавливаем ускорение и скорость
  planner.setAcceleration(100);
  planner.setMaxSpeed(300);

  // пуллапим. Кнопки замыкают на GND
  pinMode(LIMSW_X, INPUT_PULLUP);
  pinMode(LIMSW_Y, INPUT_PULLUP);
}

void loop() {
}

void homing() {
  if (digitalRead(LIMSW_X)) {       // если концевик X не нажат
    planner.setSpeed(0, -10);       // ось Х, -10 шаг/сек
    while (digitalRead(LIMSW_X)) {  // пока кнопка не нажата
      planner.tick();               // крутим
    }
    // кнопка нажалась - покидаем цикл
    planner.brake();                // тормозим, приехали
  }
  
  if (digitalRead(LIMSW_Y)) {       // если концевик Y не нажат
    planner.setSpeed(1, -10);       // ось Y, -10 шаг/сек
    while (digitalRead(LIMSW_Y)) {  // пока кнопка не нажата
      planner.tick();               // крутим
    }
    // кнопка нажалась - покидаем цикл
    planner.brake();                // тормозим, приехали
  }
  planner.reset();    // сбрасываем координаты в 0
}

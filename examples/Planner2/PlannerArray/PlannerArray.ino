// пример с записанным в памяти маршрутом
// смотри график

int path[][2] = {
  {100, 250},
  {160, 30},
  {230, 250},
  {60, 100},
  {270, 100},
};

// количество точек (пусть компилятор сам считает)
// как вес всего массива / (2+2) байта
int nodeAmount = sizeof(path) / 4;

#include "GyverPlanner2.h"
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);
GPlanner2<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(115200);
  // добавляем шаговики на оси
  planner.addStepper(0, stepper1);  // ось 0
  planner.addStepper(1, stepper2);  // ось 1

  // устанавливаем ускорение и скорость
  planner.setAcceleration(500);
  planner.setMaxSpeed(500);

  // начальная точка системы должна совпадать с первой точкой маршрута
  planner.setCurrent(path[0]);
  planner.start();
}

int count = 0;  // счётчик точек маршрута
void loop() {
  // здесь происходит движение моторов, вызывать как можно чаще
  planner.tick();

  // если в буфере планировщика есть место
  if (planner.available()) {
    // добавляем точку маршрута и является ли она точкой остановки (0 - нет)
    planner.addTarget(path[count], 0);
    if (++count >= sizeof(path) / 4) count = 0; // закольцевать
  }

  // асинхронно вывожу в порт графики
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(stepper1.pos);
    Serial.print(',');
    Serial.println(stepper2.pos);
  }
}

// пример с записанным в памяти маршрутом
// смотри график

int path[][2] = {
  {10, 10},
  {100, 5},
  {200, 200},
  {150, 150},
  {0, 0},
  {0, 50},
};

// количество точек (пусть компилятор сам считает)
// как вес всего массива / (2+2) байта
int nodeAmount = sizeof(path) / 4;

#include "GyverPlanner.h"
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);
GPlanner<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(115200);  
  // добавляем шаговики на оси
  planner.addStepper(0, stepper1);  // ось 0
  planner.addStepper(1, stepper2);  // ось 1

  // устанавливаем ускорение и скорость
  planner.setAcceleration(100);
  planner.setMaxSpeed(300);
}

int count = 0;  // счётчик точек маршрута
void loop() {
  // здесь происходит движение моторов, вызывать как можно чаще
  planner.tick();

  // вернёт true, если все моторы доехали
  if (planner.ready()) {
    
    if (count < nodeAmount) {         // ограничиваем на количество точек, чтобы не бахнуло
      planner.setTarget(path[count]); // загружаем новую точку (начнётся с 0)
      count++;
    }
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

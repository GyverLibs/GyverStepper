// пример с записанным в памяти маршрутом окружности
// работу можно посмотреть в плоттере, а лучше в приложенном stepperPlot

const int pointAm = 30;     // количество точек в круге
int radius = 100;           // радиус круга
int32_t path[pointAm + 2][2];     // буфер круга

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
  planner.setAcceleration(500);
  planner.setMaxSpeed(500);

  // заполняем буфер
  for (int i = 0; i <= pointAm; i++) {
    path[i + 1][0] = radius + radius * cos(TWO_PI * i / pointAm);
    path[i + 1][1] = radius + radius * sin(TWO_PI * i / pointAm);
  }
}

int count = 0;  // счётчик точек маршрута
void loop() {
  // здесь происходит движение моторов, вызывать как можно чаще
  planner.tick();

  // вернёт true, если все моторы доехали
  if (planner.ready()) {
    planner.setTarget(path[count]); // загружаем новую точку (начнётся с 0)
    if (++count >= sizeof(path) / 8) count = 0;
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
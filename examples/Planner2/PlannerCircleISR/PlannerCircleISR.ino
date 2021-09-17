// тож самое, но тик по таймеру
// пример с записанным в памяти маршрутом окружности
// работу можно посмотреть в плоттере, а лучше в приложенном stepperPlot для Processing

const int pointAm = 30;     // количество точек в круге
int radius = 100;           // радиус круга
int32_t path[pointAm + 1 + 1][2];     // буфер круга
// +1 на стартовую точку +1 на замыкание круга

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

  // заполняем буфер
  for (int i = 0; i <= pointAm; i++) {
    path[i + 1][0] = radius + radius * cos(TWO_PI * i / pointAm);
    path[i + 1][1] = radius + radius * sin(TWO_PI * i / pointAm);
  }
  // 0 - координата 0,0
  // 1 - первая координата круга
  // итд

  // заводим всё
  planner.start();
  initTimer();  
}

// прерывание таймера
ISR(TIMER1_COMPA_vect) {
  // здесь происходит движение моторов
  // если мотор должен двигаться (true) - ставим новый период таймеру
  if (planner.tickManual()) setPeriod(planner.getPeriod());
  else stopTimer();
  // если нет - останавливаем таймер
}

int count = 0;  // счётчик точек маршрута
void loop() {
  // вручную проверяем буфер. Если начался новый отрезок движения
  if (planner.checkBuffer()) {
    startTimer();                     // запускаем таймер
    setPeriod(planner.getPeriod());   // устанавливаем новый период
  }
  
  // если в буфере планировщика есть место
  if (planner.available()) {
    // добавляем точку маршрута и является ли она точкой остановки (0 - нет)
    planner.addTarget(path[count], 0);
    if (++count >= sizeof(path) / 8) count = 0; // закольцевать
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

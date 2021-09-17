// пример с движением моторов в прерывании
// используется Timer1 на atmega328

#include "GyverPlanner.h"
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);
GPlanner<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(115200);
  // добавляем шаговики на оси
  planner.addStepper(0, stepper1);  // ось 0
  planner.addStepper(1, stepper2);  // ось 1

  initTimer();
  startTimer();
}

// прерывание таймера
ISR(TIMER1_COMPA_vect) {
  // здесь происходит движение моторов
  // если мотор должен двигаться (true) - ставим новый период таймеру
  if (planner.tickManual()) setPeriod(planner.getPeriod());
  else stopTimer();
  // если нет - останавливаем таймер
}

int path[][2] = {
  {10, 10},
  {100, 5},
  {200, 200},
  {150, 150},
  {0, 0},
  {0, 50},
};

int count = 0;  // счётчик точек маршрута

void loop() {
  // вернёт true, если все моторы доехали
  if (planner.ready()) {
    if (count < sizeof(path) / 4) {         // ограничиваем на количество точек
      if (planner.setTarget(path[count])) { // загружаем новую точку (начнётся с 0)
        // выполняем дальше код, если есть куда двигаться (получили true)
        // после вызова setTarget обновляется период! можно его юзать
        startTimer();                   // запускаем таймер
        setPeriod(planner.getPeriod()); // устанавливаем период
        count++;
      }
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

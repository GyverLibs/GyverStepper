// здесь у нас моторы движутся по трём точкам траектории
// можно открыть плоттер, наблюать за этим и отправлять команды:
// s - стоп
// b - тормоз
// p - пауза
// r - продолжить

#include "GyverPlanner.h"
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);
GPlanner<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(57600);
  // добавляем шаговики на оси
  planner.addStepper(0, stepper1);  // ось 0
  planner.addStepper(1, stepper2);  // ось 1

  // устанавливаем ускорение и скорость
  planner.setAcceleration(200);
  planner.setMaxSpeed(200);
}

byte count = 0;
int32_t path[][2] = {
  {0, 0},
  {100, 150},
  {200, 200},
};

void loop() {
  // здесь происходит движение моторов, вызывать как можно чаще
  planner.tick();

  // вернёт true, если все моторы доехали
  if (planner.ready()) {
    planner.setTarget(path[count]); // загружаем новую точку (начнётся с 0)
    if (++count >= sizeof(path) / 8) count = 0;
  }

  // управляем процессом
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    switch (incoming) {
      case 's': planner.stop(); break;
      case 'b': planner.brake(); break;
      case 'r': planner.resume(); break;
      case 'p': planner.pause(); break;
    }
  }

  // асинхронно вывожу в порт графики
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(planner.getTarget(0));
    Serial.print(',');
    Serial.print(planner.getTarget(1));
    Serial.print(',');
    Serial.print(stepper1.pos);
    Serial.print(',');
    Serial.println(stepper2.pos);
  }
}

// базовый пример: как создать и запустить планировщик
// при запуске моторы будут отправлены на первую позицию
// при достижении - на вторую. После этого движение прекратится
// открой плоттер и смотри графики

#include "GyverPlanner.h"
// создаём моторы класса Stepper с указанием типа драйвера и пинов. Примеры:
// Stepper<STEPPER2WIRE> stepper(step, dir);                        // драйвер step-dir
// Stepper<STEPPER2WIRE> stepper(step, dir, en);                    // драйвер step-dir + пин enable
// Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4);           // драйвер 4 пин
// Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4, en);       // драйвер 4 пин + enable
// Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4);      // драйвер 4 пин полушаг
// Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4, en);  // драйвер 4 пин полушаг + enable

// МОТОРЫ ДОЛЖНЫ БЫТЬ С ОДИНАКОВЫМ ТИПОМ ДРАЙВЕРА
// вот они красавцы
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);

// создаём планировщик, указываем в <> тип драйвера КАК У МОТОРОВ
// и количество осей, равное количеству моторов (любое больше 1)
GPlanner<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(115200);
  // добавляем шаговики на оси
  planner.addStepper(0, stepper1);  // ось 0
  planner.addStepper(1, stepper2);  // ось 1

  // устанавливаем ускорение и скорость
  planner.setAcceleration(100);
  planner.setMaxSpeed(300);

  planner.reset();  // сбрасываем все позиции в 0 (они и так в 0 при запуске)

  // массив с целевыми позициями осей, размер массива равен количеству осей
  int target[] = {300, 200};

  // отправляем
  planner.setTarget(target);
}

void loop() {
  // здесь происходит движение моторов, вызывать как можно чаще
  planner.tick();

  // вернёт true, если все моторы доехали
  if (planner.ready()) {
    // загружаем новую точку
    int newTarget[] = {10, 50};
    planner.setTarget(newTarget);
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

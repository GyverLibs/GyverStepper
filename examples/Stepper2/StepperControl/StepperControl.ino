// здесь у нас моторы движутся по трём точкам траектории
// можно открыть плоттер, наблюать за этим и отправлять команды:
// s - стоп
// b - тормоз
// p - пауза
// r - продолжить
//#define GS_NO_ACCEL
#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 3);

void setup() {
  Serial.begin(9600);

  // устанавливаем ускорение и скорость
  stepper.setAcceleration(200);
  stepper.setMaxSpeed(100);
  stepper.setTarget(0);
  //stepper.setSpeed(100);
}

byte count = 0;
int16_t path[] = {0, 200, 100};

void loop() {
  // здесь происходит движение мотора, вызывать как можно чаще
  stepper.tick();

  // вернёт true, если все моторы доехали
  if (stepper.ready()) {
    stepper.setTarget(path[count]); // загружаем новую точку (начнётся с 0)
    if (++count >= sizeof(path) / 2) count = 0;
  }

  // управляем процессом
  if (Serial.available() > 0) {
    char incoming = Serial.read();
    switch (incoming) {
      case 's': stepper.stop(); break;
      case 'b': stepper.brake(); break;
      case 'r': stepper.resume(); break;
      case 'p': stepper.pause(); break;
    }
  }

  // асинхронно вывожу в порт графики
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(stepper.getTarget());
    Serial.print(',');
    Serial.println(stepper.pos);
  }
}

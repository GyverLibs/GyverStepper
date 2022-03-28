// установка позиции потенциометром

#include <GyverStepper.h>
//GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);

void setup() {
  Serial.begin(115200);
  stepper.setRunMode(FOLLOW_POS);   // режим следования к целевй позиции
  stepper.setMaxSpeed(400);         // установка макс. скорости в шагах/сек
  stepper.setAcceleration(500);     // установка ускорения в шагах/сек/сек

  // пусть драйвер выключается при достижении позиции
  stepper.autoPower(true);
}

void loop() {
  stepper.tick();

  // сделаем таймер на 20 мс
  // будем опрашивать потенциометр и строить графики
  static uint32_t tmr2;
  if (millis() - tmr2 > 20) {
    tmr2 = millis();
    static float val;
    // потенциометр на A0
    // фильтруем, иначе мотор будет трястись
    val += (analogRead(0) - val) * 0.08;

    stepper.setTarget(val);     // ставим новую позицию
    Serial.print(stepper.getTarget());
    Serial.print(',');
    Serial.println(stepper.getCurrent());
  }
}

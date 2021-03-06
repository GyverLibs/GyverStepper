// начинаем плавный разгон при удержании кнопки. При отпускании тормозим

//#define SMOOTH_ALGORITHM
#include <GyverStepper.h>
//GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);


void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT_PULLUP);  // кнопка на A0 и GND
  stepper.enable();
  stepper.autoPower(true);

  // установка ускорения в шагах/сек/сек
  stepper.setAcceleration(1200);
  stepper.setRunMode(KEEP_SPEED);
}

bool btnState = false;
void loop() {
  stepper.tick();

  // кнопка нажата
  if (!digitalRead(A0) && !btnState) {
    btnState = true;
    stepper.setSpeed(3000, SMOOTH);
  }

  // кнопка отпущена
  if (digitalRead(A0) && btnState) {
    btnState = false;
    stepper.stop();
  }
}

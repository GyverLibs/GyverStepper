// пример с концевиком
#include <GyverStepper.h>
//GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);
GStepper<STEPPER2WIRE> stepper(2048, 2, 5);

void setup() {
  // наша задача - при запуске крутить мотор в сторону до нажатия на кнопку
  pinMode(12, INPUT_PULLUP);  // кнопка на D12 и GND
  
  stepper.setRunMode(KEEP_SPEED);
  stepper.setSpeedDeg(-10);   // медленно крутимся НАЗАД

  // пока кнопка не нажата
  while(digitalRead(12)) {    
    stepper.tick();
	// yield();	// для esp8266
  }
  // вот тут кнопка нажата, сразу вырубаем мотор.
  // Текущее положение также сбрасывается в 0
  stepper.reset();

  // дальше например врубаем FOLLOW_POS
  stepper.setRunMode(FOLLOW_POS);
}

void loop() {
  // и качаемся в 20 шагах от кнопки и до 300
  static bool dir;
  if (!stepper.tick()) {
    dir = !dir;
    stepper.setTarget(dir ? 20 : 300);
  }
}

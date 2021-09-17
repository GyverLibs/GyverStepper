![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)
![author](https://img.shields.io/badge/author-AlexGyver-informational.svg)
# GyverStepper
Производительная библиотека для управления шаговыми моторами с Arduino
- Поддержка 4х пинового (шаг и полушаг) и STEP-DIR драйверов
- Автоматическое отключение питания при достижении цели
- Режимы работы:
    - Вращение с заданной скоростью. Плавный разгон и торможение с ускорением
    - Следование к позиции с ускорением и ограничением скорости
    - Следование к позиции с заданной скоростью (без ускорения)
- Быстрый алгоритм управления шагами
- Два алгоритма плавного движения
    - Мой планировщик обеспечивает максимальную производительность: 
    скорость до 30'000 шагов/сек с ускорением (активен по умолчанию)
    - Модифицированный планировщик из AccelStepper: максимальную плавность и 
    скорость до 7'000 шагов/сек с ускорением (для активации пропиши дефайн SMOOTH_ALGORITHM)
- Поддержка "виртуальных" драйверов
- Встроенный многоосевой планировщик траекторий

### Совместимость
Совместима со всеми Arduino платформами (используются Arduino-функции)

### Документация
К библиотеке есть [расширенная документация](https://alexgyver.ru/GyverStepper/)

## Содержание
- [Установка](#install)
- [GyverStepper](#stepper)
- [StepperCore](#core)
- [GyverPlanner](#planner)
- [GyverPlanner2](#planner2)
- [Версии](#versions)
- [Баги и обратная связь](#feedback)

<a id="install"></a>
## Установка
- Библиотеку можно найти по названию **GyverStepper** и установить через менеджер библиотек в:
    - Arduino IDE
    - Arduino IDE v2
    - PlatformIO
- [Скачать библиотеку](https://github.com/GyverLibs/GyverStepper/archive/refs/heads/main.zip) .zip архивом для ручной установки:
    - Распаковать и положить в *C:\Program Files (x86)\Arduino\libraries* (Windows x64)
    - Распаковать и положить в *C:\Program Files\Arduino\libraries* (Windows x32)
    - Распаковать и положить в *Документы/Arduino/libraries/*
    - (Arduino IDE) автоматическая установка из .zip: *Скетч/Подключить библиотеку/Добавить .ZIP библиотеку…* и указать скачанный архив
- Читай более подробную инструкцию по установке библиотек [здесь](https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)

<a id="stepper"></a>
## GyverStepper
### Инициализация (GyverStepper)
```cpp
// steps - шагов на один оборот вала (для расчётов с градусами)
// step, dir, pin1, pin2, pin3, pin4 - любые GPIO
// en - пин отключения драйвера, любой GPIO
GStepper<STEPPER2WIRE> stepper(steps, step, dir);                   // драйвер step-dir
GStepper<STEPPER2WIRE> stepper(steps, step, dir, en);               // драйвер step-dir + пин enable
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4);      // драйвер 4 пин
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4, en);  // драйвер 4 пин + enable
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4);     // драйвер 4 пин полушаг
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4, en); // драйвер 4 пин полушаг + enable
GStepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper(2048);                  // виртуальный драйвер, указать только количество шагов
```

### Использование (GyverStepper)
```cpp
// Примечание: далее по тексту под "по умолчанию" имеется в виду "даже если не вызывать функцию"

// Здесь происходит движение мотора, вызывать как можно чаще!
// Имеет встроенный таймер
// Возвращает true, если мотор движется к цели или крутится по KEEP_SPEED
bool tick();

// Возвращает то же самое, что tick, т.е. крутится мотор или нет
bool getState();

// Инвертировать направление мотора - true (по умолч. false)
void reverse(bool dir);

// инвертировать поведение EN пина - true (по умолч. false)
void invertEn(bool rev);

// Установка режима работы, mode:
// FOLLOW_POS - следование к позиции setTarget(...)
// KEEP_SPEED - удержание скорости setSpeed(...)
void setRunMode(GS_runMode mode);

// Установка текущей позиции мотора в шагах и градусах
void setCurrent(long pos);
void setCurrentDeg(float pos);

// Чтение текущей позиции мотора в шагах и градусах
long getCurrent();
float getCurrentDeg();

// установка целевой позиции в шагах и градусах (для режима FOLLOW_POS)
// type - ABSOLUTE или RELATIVE, по умолчанию стоит ABSOLUTE
void setTarget(long pos);
void setTarget(long pos, GS_posType type);
void setTargetDeg(float pos);
void setTargetDeg(float pos, GS_posType type);

// Получение целевой позиции в шагах и градусах
long getTarget();
float getTargetDeg();

// Установка максимальной скорости (по модулю) в шагах/секунду и градусах/секунду (для режима FOLLOW_POS)
// по умолч. 300
void setMaxSpeed(float speed);
void setMaxSpeedDeg(float speed);

// Установка ускорения в шагах и градусах в секунду (для режима FOLLOW_POS).
// При значении 0 ускорение отключается и мотор работает 
// по профилю постоянной максимальной скорости setMaxSpeed().
// По умолч. 300
void setAcceleration(int accel);
void setAccelerationDeg(float accel);

// Автоотключение EN при достижении позиции - true (по умолч. false)
void autoPower(bool mode);

// Плавная остановка с заданным ускорением
void stop();

// Жёсткая остановка
void brake();

// Жёсткая остановка + сброс позиции в 0 (для концевиков)
void reset();

// Установка целевой скорости в шагах/секунду и градусах/секунду (для режима KEEP_SPEED)
void setSpeed(float speed);
void setSpeedDeg(float speed);

// Получение целевой скорости в шагах/секунду и градусах/секунду (для режима KEEP_SPEED)
float getSpeed();
float getSpeedDeg();

// Включить мотор (пин EN)
void enable();

// Выключить мотор (пин EN)
void disable();

// Возвращает минимальный период тика мотора в микросекундах при настроенной setMaxSpeed() скорости.
// Можно использовать для настройки прерываний таймера, в обработчике которого будет лежать tick() (см. пример timerISR)
uint16_t getMinPeriod();

// Текущий период "тика" для отладки и всего такого
uint16_t stepTime;

// подключить внешний обработчик для шага и переключения питания
void attachStep(handler)
void attachPower(handler)
```

### Пример (GyverStepper)
1. First item with a hidden code block
<details>
  <summary>Example</summary>

  ```
  long console output here
  ```
</details>

<details>
  <summary>Example</summary>
Остальные примеры смотри в **examples**!
```cpp
    #include <GyverStepper.h>
    GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);

    void setup() {
      Serial.begin(115200);
      // режим поддержания скорости
      stepper.setRunMode(KEEP_SPEED);

      // можно установить скорость
      stepper.setSpeed(120);    // в шагах/сек
      stepper.setSpeedDeg(80);  // в градусах/сек

      // режим следования к целевй позиции
      stepper.setRunMode(FOLLOW_POS);

      // можно установить позицию
      stepper.setTarget(-2024);    // в шагах
      stepper.setTargetDeg(-360);  // в градусах

      // установка макс. скорости в градусах/сек
      stepper.setMaxSpeedDeg(400);
      
      // установка макс. скорости в шагах/сек
      stepper.setMaxSpeed(400);

      // установка ускорения в градусах/сек/сек
      stepper.setAccelerationDeg(300);

      // установка ускорения в шагах/сек/сек
      stepper.setAcceleration(300);

      // отключать мотор при достижении цели
      stepper.autoPower(true);

      // включить мотор (если указан пин en)
      stepper.enable();
    }

    void loop() {
      // просто крутим туды-сюды
      if (!stepper.tick()) {
        static bool dir;
        dir = !dir;
        stepper.setTarget(dir ? -1024 : 1024);
      }
    }
```
</details>

123
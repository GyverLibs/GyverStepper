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
- Поддержка "виртуальных" драйверов
- Встроенный многоосевой планировщик траекторий

### Совместимость
Совместима со всеми Arduino платформами (используются Arduino-функции)

### Документация
К библиотеке есть [расширенная документация](https://alexgyver.ru/GyverStepper/)

## Содержание
- [Установка](#install)
- [StepperCore](#core)
- [GyverStepper](#stepper)
- [GyverStepper2](#stepper2)
- [GyverPlanner](#planner)
- [GyverPlanner2](#planner2)
- [Версии](#versions)
- [Баги и обратная связь](#feedback)


## Аааа почему так много всего?!
Библиотека содержит набор инструментов для разных сценариев работы с шаговыми моторами
- *StepperCore.h* [класс **Stepper**]: ядро всех остальных классов, умеет быстро щёлкать пинами (AVR) и делать один шаг для настроенного типа драйвера. Поддерживает 4 фазы шаг/полушаг, а также step-dir драйверы.
- *GyverStepper.h* [класс **GStepper**]: основная тяжёлая библиотека, много настроек. Движение одного мотора с ускорением к заданной позиции или вращение с заданной скоростью. Не очень оптимальная работа в прерывании таймера.
- *GyverStepper2.h* [класс **GStepper2**]: новая облегченная версия GyverStepper, практически полностью с ней совместима. Более оптимальный целочисленный гибридный алгоритм движения с ускорением, лёгкий вес. Оптимизировано для работы в прерывании таймера.
- *GyverPlanner.h* [класс **GPlanner**]: многоосевой планировщик траектории, движение с ускорением (2 порядок). Остановка в каждой точке. Оптимальная работа в прерывании таймера.
- *GyverPlanner2.h* [класс **GPlanner2**]: многоосевой планировщик траектории, движение с ускорением (2 порядок). Планирование скорости на маршруте, оптимальное движение по точкам. Оптимальная работа в прерывании таймера.

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

<a id="core"></a>
## StepperCore
### Описание
Ядро библиотеки для управления шаговыми моторами:
- 4 фазные и STEP DIR драйверы
- Поддержка пина EN
- Виртуальный драйвер
- Быстрый алгоритм IO для AVR

<details>
<summary>РАЗВЕРНУТЬ</summary>

### Инициализация (StepperCore)
```cpp
Stepper<STEPPER2WIRE> stepper(step, dir);						// драйвер step-dir
Stepper<STEPPER2WIRE> stepper(step, dir, en);					// драйвер step-dir + пин enable
Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4);			// драйвер 4 пин
Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4, en);		// драйвер 4 пин + enable
Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4);		// драйвер 4 пин полушаг
Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4, en);	// драйвер 4 пин полушаг + enable

Stepper<STEPPER2WIRE, STEPPER_VIRTUAL> stepper;					// виртуальный драйвер step-dir
Stepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper;					// виртуальный драйвер 4 пин
```

### Использование (StepperCore)
```cpp
// настроить пины
void setPins(uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255, uint8_t pin4 = 255, uint8_t pin5 = 255);

void step();                                // сделать шаг
void invertEn(bool val);                    // инвертировать поведение EN пина
void reverse(bool val);                     // инвертировать направление мотора
void disable();                             // отключить питание и EN
void enable();                              // включить питание и EN
void attachStep(void (*handler)(uint8_t));  // подключить обработчик шага
void attachPower(void (*handler)(bool));    // подключить обработчик питания

int32_t pos;                                // текущая позиция в шагах
int8_t dir;                                 // направление (1, -1)
```

### Пример
Остальные примеры смотри в **examples**!
```cpp
#include <StepperCore.h>
Stepper<STEPPER2WIRE> stepper(2, 3);

void setup() {
  stepper.dir = 1;  // или -1
  stepper.pos = 0;  // доступ к позиции
}

void loop() {
  // крутим вручную
  stepper.step();   // сделать шаг
  delay(10);
}
```
</details>

<a id="stepper"></a>
## GyverStepper
### Описание
Библиотека для управления шаговыми моторами с Arduino
- Поддержка 4х пинового (шаг и полушаг) и STEP-DIR драйверов
- Автоматическое отключение питания при достижении цели
- Режимы работы:
    - Вращение с заданной скоростью. Плавный разгон и торможение с ускорением
    - Следование к позиции с ускорением и ограничением скорости
    - Следование к позиции с заданной скоростью (без ускорения)

<details>
<summary>РАЗВЕРНУТЬ</summary>

### Инициализация
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

### Использование
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

### Пример
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

<a id="stepper2"></a>
## GyverStepper2
### Описание
Облегчённая GyverStepper
- Легче на несколько кБ, всё целочисленное
- Повышенная точность позиционирования
- Более эффективный гибридный алгоритм движения
- Движение к цели с ускорением
- Движение от точки к точке. Смена точки во время движения не будет плавной
- Вращение со скоростью (без плавной смены скорости)
- Макс. скорость: 37000 шаг/с на полной, 18000 шаг/с на разгоне
- Оптимизировано для работы по прерыванию таймера
- Наследует StepperCore

<details>
<summary>РАЗВЕРНУТЬ</summary>

### Инициализация
```cpp
GStepper2<STEPPER2WIRE> stepper(шаговНаОборот, step, dir);                          // драйвер step-dir
GStepper2<STEPPER2WIRE> stepper(шаговНаОборот, step, dir, en);                      // драйвер step-dir + пин enable
GStepper2<STEPPER4WIRE> stepper(шаговНаОборот, pin1, pin2, pin3, pin4);             // драйвер 4 пин
GStepper2<STEPPER4WIRE> stepper(шаговНаОборот, pin1, pin2, pin3, pin4, en);         // драйвер 4 пин + enable
GStepper2<STEPPER4WIRE_HALF> stepper(шаговНаОборот, pin1, pin2, pin3, pin4);        // драйвер 4 пин полушаг
GStepper2<STEPPER4WIRE_HALF> stepper(шаговНаОборот, pin1, pin2, pin3, pin4, en);    // драйвер 4 пин полушаг + enable

GStepper2<STEPPER2WIRE, STEPPER_VIRTUAL> stepper;    // виртуальный драйвер step-dir
GStepper2<STEPPER4WIRE, STEPPER_VIRTUAL> stepper;    // виртуальный драйвер 4 пин
```

### Использование
```cpp
// === наследуется из Stepper ====
// настроить пины
void setPins(uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255, uint8_t pin4 = 255, uint8_t pin5 = 255);

void step();                                // сделать шаг
void invertEn(bool val);                    // инвертировать поведение EN пина
void reverse(bool val);                     // инвертировать направление мотора
void disable();                             // отключить питание и EN
void enable();                              // включить питание и EN
void attachStep(void (*handler)(uint8_t));  // подключить обработчик шага
void attachPower(void (*handler)(bool));    // подключить обработчик питания

int32_t pos;                                // текущая позиция в шагах
int8_t dir;                                 // направление (1, -1)

// ========= GStepper2 ==========
// тикер
bool tick();                                // тикер движения, вызывать часто. Вернёт true, если мотор движется
bool tickManual();                          // ручной тикер для вызова в прерывании таймера с периодом getPeriod(). Вернёт true, если мотор движется
bool ready();                               // однократно вернёт true, если мотор доехал до установленной позиции и остановился

// вращение
void setSpeed(int16_t speed);               // установить скорость в шагах/сек и запустить вращение
void setSpeed(float speed);                 // установить скорость в шагах/сек (float) и запустить вращение

// движение к цели
void setTarget(int32_t ntar, GS_posType type = ABSOLUTE);       // установить цель в шагах и опционально режим ABSOLUTE/RELATIVE
void setTargetDeg(int32_t ntar, GS_posType type = ABSOLUTE);    // установить цель в градусах и опционально режим ABSOLUTE/RELATIVE
int32_t getTarget();                                            // получить целевую позицию в шагах

void setAcceleration(uint16_t nA);          // установка ускорения в шаг/сек^2
void setMaxSpeed(int speed);                // установить скорость движения при следовании к позиции setTarget() в шагах/сек
void setMaxSpeed(float speed);              // установить скорость движения при следовании к позиции setTarget() в шагах/сек, float
void setMaxSpeedDeg(int speed);             // установить скорость движения при следовании к позиции в град/сек
void setMaxSpeedDeg(float speed);           // установить скорость движения при следовании к позиции в град/сек, float

void setCurrent(int32_t npos);              // установить текущую позицию
int32_t getCurrent();                       // получить текущую позицию
void reset();                               // сбросить текущую позицию в 0

// всякое
uint32_t getPeriod();                       // получить текущий период тиков
void brake();                               // резко остановить мотор
void pause();                               // пауза - доехать до заданной точки и ждать (ready() не вернёт true, пока ты на паузе)
void resume();                              // продолжить движение после остановки/паузы
uint8_t getStatus();                        // текущий статус: 0 - стоим, 1 - едем, 2 - едем к точке паузы, 3 - крутимся со скоростью, 4 - тормозим

// ===== ДЕФАЙНЫ НАСТРОЕК =====
// дефайнить перед подключением библиотеки
#define GS_NO_ACCEL                         // отключить модуль движения с ускорением (уменьшить вес кода)
```

### Пример
Остальные примеры смотри в **examples**!
```cpp
// крутим туда сюда, тикаем в loop

#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 3);

void setup() {
  Serial.begin(9600);
  //stepper.enable();
  stepper.setMaxSpeed(100);     // скорость движения к цели
  stepper.setAcceleration(200); // ускорение
  stepper.setTarget(300);       // цель
}

bool dir = 1;
void loop() {
  stepper.tick();   // мотор асинхронно крутится тут

  // если приехали
  if (stepper.ready()) {
    dir = !dir;   // разворачиваем
    stepper.setTarget(dir * 300); // едем в другую сторону
  }

  // асинхронный вывод в порт
  static uint32_t tmr;
  if (millis() - tmr >= 30) {
    tmr = millis();
    Serial.println(stepper.pos);
  }
}
```
</details>


<a id="planner"></a>
## GyverPlanner
### Описание
Многоосевой планировщик траекторий для шаговых моторов и создания станка с ЧПУ
- ОСТАНОВКА В КАЖДОЙ ТОЧКЕ. БУФЕР НА ОДНУ СЛЕДУЮЩУЮ ПОЗИЦИЮ
- Макс. скорость: 37000 шаг/с на полной, 14000 шаг/с на разгоне
- Трапецеидальный профиль скорости (планировщик 2-го порядка)
- Настройка скорости и ускорения
- Любое количество осей. Будут двигаться синхронно к заданным целям
- Быстрая целочисленная модель планирования траектории и скорости    
- Режим постоянного вращения для одной оси (для движения к концевику например)
- Тормоз/плавная остановка/пауза на траектории планировщика
- Оптимизировано для работы по прерыванию таймера
- Быстрый контроль пинов шаговика для Arduino AVR

### Логика работы
Планировщик управляет любым количеством моторов, вращая их к указанной позиции. В данной версии 
остановка происходит в каждой точке траектории, после чего поднимается флаг ready() и ожидает 
установки следующей точки.
- Смотри симуляцию в Processing: папка Planner Simulation/Planner

<details>
<summary>РАЗВЕРНУТЬ</summary>
    
### Инициализация
```cpp
GPlanner<драйвер, количество осей> planner;
```

### Использование
```cpp
void addStepper(uint8_t axis, Stepper &stp);    // подключить мотор класса Stepper на ось axis
// примечание: тип драйвера должен совпадать у планировщика и моторов

// НАСТРОЙКИ
void setMaxSpeed(float nV);                 // установка максимальной скорости планировщика в шаг/сек
void setAcceleration(uint16_t nA);          // установка ускорения планировщика в шаг/сек^2

// ПЛАНИРОВЩИК
uint32_t getPeriod();                       // возвращает время в мкс до следующего вызова tick/tickManual
bool ready();                               // true - готов принять следующую точку маршрута
void pause();                               // пауза (доехать до заданной точки и ждать). ready() не вернёт true, пока ты на паузе
void stop();                                // остановить плавно (с заданным ускорением)
void brake();                               // резко остановить моторы из любого режима
void resume();                              // продолжить после остановки/паузы
void reset();                               // сбросить счётчики всех моторов в 0
uint8_t getStatus();                        // текущий статус: 0 - стоим, 1 - едем, 2 - едем к точке паузы, 3 -крутимся со скоростью 

// СКОРОСТЬ
void setSpeed(uint8_t axis, float speed);   // режим постоянного вращения для оси axis со скоростью speed шаг/сек (м.б. отрицателеьной)

// ПОЗИЦИЯ
void setCurrent(int16_t cur[]);             // установить текущее положение моторов
void setCurrent(int32_t cur[]);             // установить текущее положение моторов
int32_t getCurrent(int axis);               // получить текущую позицию по оси axis

// установить цель в шагах и начать движение. type - ABSOLUTE (по умолч.) или RELATIVE
// ABSOLUTE - конкретные координаты точки, куда двигаться
// RELATIVE - смещение относительно текущих положений моторов
// вернёт true, если цель установлена. false, если цель совпадает с текущей
bool setTarget(int32_t target[]);
bool setTarget(int16_t target[]);
bool setTarget(int32_t target[], type);
bool setTarget(int16_t target[], type);
int32_t getTarget(int axis);                // получить цель в шагах на оси axis

// ТИКЕР
// тикер, вызывать как можно чаще. Вернёт true, если мотор крутится
// здесь делаются шаги как для движения по точкам, так и для вращения по скорости
bool tick();

// ручной тикер для вызова в прерывании или где то ещё. Выполняется 20..50 us
bool tickManual();
```

### Пример
Остальные примеры смотри в **examples**!
```cpp
// базовый пример: как создать и запустить планировщик
// при запуске моторы будут отправлены на первую позицию
// при достижении - на вторую. После этого движение прекратится
// открой плоттер и смотри графики

#include "GyverPlanner.h"
// создаём моторы класса Stepper с указанием типа драйвера и пинов
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
```
</details>

<a id="planner2"></a>
## GyverPlanner2
### Описание
Многоосевой планировщик траекторий для шаговых моторов и создания станка с ЧПУ
- ПЛАНИРОВАНИЕ СКОРОСТИ НА МАРШРУТЕ. НАСТРАИВАЕМЫЙ БУФЕР
- Макс. скорость: 37000 шаг/с на полной, 14000 шаг/с на разгоне
- Трапецеидальный профиль скорости (планировщик 2-го порядка)
- Настройка скорости и ускорения
- Любое количество осей. Будут двигаться синхронно к заданным целям
- Быстрая целочисленная модель планирования траектории и скорости    
- Режим постоянного вращения для одной оси (для движения к концевику например)
- Тормоз/плавная остановка/пауза на траектории планировщика
- Оптимизировано для работы по прерыванию таймера
- Быстрый контроль пинов шаговика для Arduino AVR

### Логика работы
Планировщик управляет любым количеством моторов, вращая их к указанной позиции. В данной версии 
реализован буфер траектории, который можно наполнять точками, пока available() возвращает true. 
addTarget() принимает:
- Массив точек указанного при инициализации размера
- Флаг остановки. Если передать 1 - планировщик остановит мотор в этой точке и будет ждать дальнейшей команды resume()
- Тип точки: ABSOLUTE (абсолютная координата) или RELATIVE (относительно предыдущей точки)

Когда плаанировщик приезжает до точки остановки - он встаёт на паузу (например для включения выключения инструмента), 
после совершения нужных действий вызываем resume() и он продолжает движение.  
В отличие от предыдущего GPlanner, в GPlanner2 реализован просчёт траектории в буфере и планирование 
скорости для всех точек, что позволяет системе двигаться быстрее и не тормозить в каждой точке.
- Смотри симуляцию в Processing: папка Planner Simulation/Planner2

<details>
<summary>РАЗВЕРНУТЬ</summary>

### Инициализация
```cpp
GPlanner<драйвер, количество осей> planner;                 // объявяление
GPlanner<драйвер, количество осей, размер буфера> planner;  // + размер буфера (по умолч. 32)
```

### Использование
```cpp
void addStepper(uint8_t axis, Stepper &stp);    // подключить мотор класса Stepper на ось axis
// примечание: тип драйвера должен совпадать у планировщика и моторов

// НАСТРОЙКИ
void setMaxSpeed(float nV);                 // установка максимальной скорости планировщика в шаг/сек
void setAcceleration(uint16_t nA);          // установка ускорения планировщика в шаг/сек^2
void setDtA(float newDta);                  // установить dt смены скорости в повороте, 0.0.. 1.0 по умолч. 0.3

// ПЛАНИРОВЩИК
uint32_t getPeriod();                       // возвращает время в мкс до следующего вызова tick/tickManual
void start();                               // начать работу
void stop();                                // остановить плавно (с заданным ускорением)
void brake();                               // резко остановить моторы из любого режима
void resume();                              // продолжить после остановки или конечной точки маршрута
void reset();                               // сбросить счётчики всех моторов в 0
bool ready();                               // флаг достижения точки остановки. После неё нужно вызывать resume
bool available();                           // true - в буфере планировщика есть место под новю точку

uint8_t getStatus();                        // текущий статус:
// 0 ожидание команды (остановлен)
// 1 ожидание буфера
// 2 в пути
// 3 на паузу
// 4 на стоп
// 5 крутится setSpeed

// СКОРОСТЬ
void setSpeed(uint8_t axis, float speed);   // режим постоянного вращения для оси axis со скоростью speed шаг/сек (м.б. отрицателеьной)

// ПОЗИЦИЯ
// добавить новую точку маршрута. Массив координат, флаг окончания и абсолютный/относительный
void addTarget(int32_t tar[], uint8_t l, GS_posType type = ABSOLUTE);
void addTarget(int16_t tar[], uint8_t l, GS_posType type = ABSOLUTE);
// ABSOLUTE - конкретные координаты точки, куда двигаться
// RELATIVE - смещение относительно текущих положений моторов

void setCurrent(int16_t cur[]);             // установить текущее положение моторов
void setCurrent(int32_t cur[]);             // установить текущее положение моторов
int32_t getCurrent(int axis);               // получить текущую позицию по оси axis
int32_t getTarget(int axis);                // получить текущую цель в шагах на оси axis

// ТИКЕР
// тикер, вызывать как можно чаще. Вернёт true, если мотор крутится
// здесь делаются шаги для движения по точкам, для вращения по скорости, а также перестройка буфера
bool tick();

// ручной тикер для вызова в прерывании или где то ещё. Выполняется 20..50 us
bool tickManual();

// обработчик буфера. Сам вызывается в tick. Нужно вызывать вручную при работе с tickManual
// вернёт true, если планировщик отправил моторы на новую позицию (в этот момент можно запускать таймер)
void checkBuffer();
```

### Пример
Остальные примеры смотри в **examples**!
```cpp
// пример с записанным в памяти маршрутом
// смотри график, а лучше запусти stepperPlot

int path[][2] = {
  {100, 250},
  {160, 30},
  {230, 250},
  {60, 100},
  {270, 100},
};

// количество точек (пусть компилятор сам считает)
// как вес всего массива / (2+2) байта
int nodeAmount = sizeof(path) / 4;

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
  planner.start();
}

int count = 0;  // счётчик точек маршрута
void loop() {
  // здесь происходит движение моторов, вызывать как можно чаще
  planner.tick();

  // если в буфере планировщика есть место
  if (planner.available()) {
    // добавляем точку маршрута и является ли она точкой остановки (0 - нет)
    planner.addTarget(path[count], 0);
    if (++count >= sizeof(path) / 4) count = 0; // закольцевать
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
```
</details>

<a id="versions"></a>
## Версии
- v1.1 - добавлена возможность плавного управления скоростью в KEEP_SPEED (см. пример accelDeccelButton)
- v1.2 - добавлена поддержка ESP8266
- v1.3 - изменена логика работы setTarget(, RELATIVE)
- v1.4 - добавлена задержка для STEP, настроить можно дефайном DRIVER_STEP_TIME
- v1.5 - пофикшен баг для плат есп
- v1.6 - Исправлена остановка для STEPPER4WIRE_HALF, скорость можно задавать во float (для медленных скоростей)
- v1.7 - Исправлен баг в отрицательной скорости (спасибо Евгению Солодову)
- v1.8 - Исправлен режим KEEP_SPEED
- v1.9 - Исправлена ошибка с esp функцией max
- v1.10 - повышена точность
- v1.11 - повышена точность задания скорости
- v1.12 - пофикшена плавная работа в KEEP_SPEED. Добавлена поддержка "внешних" драйверов. Убран аргумент SMOOTH из setSpeed
- v1.13 - исправлены мелкие баги, оптимизация
- v1.14 - исправлены ошибки разгона и торможения в KEEP_SPEED
- v1.15 - оптимизация, исправлены мелкие баги, stop() больше не сбрасывает maxSpeed
- v1.15.2 - добавил включение EN если указан, даже при отключенном autoPower
- v2.0 - оптимизация. Ядро шаговика вынесено в отдельный класс Stepper. Добавлены многоосевые планировщики траекторий
- v2.1 - добавил GyverStepper2, упрощённая и оптимизированная версия GyverStepper

<a id="feedback"></a>
## Баги и обратная связь
При нахождении багов создавайте **Issue**, а лучше сразу пишите на почту [alex@alexgyver.ru](mailto:alex@alexgyver.ru)  
Библиотека открыта для доработки и ваших **Pull Request**'ов!
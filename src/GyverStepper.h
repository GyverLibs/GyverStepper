/*
    Производительная библиотека для управления шаговыми моторами с Arduino
    Документация: https://alexgyver.ru/gyverstepper/
    GitHub: https://github.com/GyverLibs/GyverStepper
    Возможности:
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

    Алгоритм из AccelStepper: https://www.airspayce.com/mikem/arduino/AccelStepper/
    AlexGyver, alex@alexgyver.ru
    https://alexgyver.ru/
    MIT License

    Версии:
    v1.1 - добавлена возможность плавного управления скоростью в KEEP_SPEED (см. пример accelDeccelButton)
    v1.2 - добавлена поддержка ESP8266
    v1.3 - изменена логика работы setTarget(, RELATIVE)
    v1.4 - добавлена задержка для STEP, настроить можно дефайном DRIVER_STEP_TIME
    v1.5 - пофикшен баг для плат есп
    v1.6 - Исправлена остановка для STEPPER4WIRE_HALF, скорость можно задавать во float (для медленных скоростей)
    v1.7 - Исправлен баг в отрицательной скорости (спасибо Евгению Солодову)
    v1.8 - Исправлен режим KEEP_SPEED
    v1.9 - Исправлена ошибка с esp функцией max
    v1.10 - повышена точность
    v1.11 - повышена точность задания скорости
    v1.12 - пофикшена плавная работа в KEEP_SPEED. Добавлена поддержка "внешних" драйверов. Убран аргумент SMOOTH из setSpeed
    v1.13 - исправлены мелкие баги, оптимизация
    v1.14 - исправлены ошибки разгона и торможения в KEEP_SPEED
    v1.15 - оптимизация, исправлены мелкие баги, stop() больше не сбрасывает maxSpeed
    v1.15.2 - добавил включение EN если указан, даже при отключенном autoPower
    v2.0 - оптимизация. Ядро шаговика вынесено в отдельный класс Stepper. Добавлены многоосевые планировщики траекторий
    v2.1 - добавил GyverStepper2, упрощённая и оптимизированная версия GyverStepper
    v2.1.1 - исправлена бага в GyverStepper
    v2.1.2 - совместимость Digispark
    v2.1.3 - починил FOLLOW_POS в GStepper, починил RELATIVE в GPlanner2 и исправил багу с рывками
    v2.1.4 - GPlanner2: исправил рывки, добавил адаптивное перестроение траектории без остановок, чутка оптимизировал вычисления
    v2.1.5 - возможность менять скорость и ускорение во время работы планировщика (GStepper2, GPlanner, GPlanner2)
    v2.1.6 - исправлена ошибка компиляции при вызове disable() в GStepper
    v2.1.7 - добавлен clearBuffer() в GPlanner2
    v2.1.8 - оптимизация, исправлен KEEP_SPEED в GStepper
    v2.2.0 - добавлен скоростной профиль GS_FAST_PROFILE для GStepper2, GPlanner, GPlanner2. Поддержка режима "слежения" для GStepper2
    v2.2.1 - небольшая оптимизация SRAM
    v2.3 - fix compiler warnings, поддержка esp32
    v2.4 - повышена плавность движения шаговиков в Planner и Planner2. Исправлена бага в Stepper2
    v2.5 - исправлено плавное изменение скорости для KEEP_SPEED
    v2.6
        - disable() в виртуальном режиме отключает сигнал с мотора (для 4-проводных драйверов)
        - улучшена производительность для step-dir драйверов
        - добавил autoPower() в GStepper2
        - исправлен рывок при смене направления в GStepper
    v2.6.1 - поправлена бага в GStepper2
    v2.6.2 - оптимизированы вычисления в GStepper2, GPlanner и GPlanner2
    v2.6.3 - reverse() в step-dir драйвере теперь применяется сразу
*/

/*
// Примечание: далее по тексту под "по умолчанию" имеется в виду "даже если не вызывать функцию"

// Создание объекта
// steps - шагов на один оборот вала (для расчётов с градусами)
// step, dir, pin1, pin2, pin3, pin4 - любые GPIO
// en - пин отключения драйвера, любой GPIO
GStepper<STEPPER2WIRE> stepper(steps, step, dir);                        // драйвер step-dir
GStepper<STEPPER2WIRE> stepper(steps, step, dir, en);                    // драйвер step-dir + пин enable
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4);            // драйвер 4 пин
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4, en);        // драйвер 4 пин + enable
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4);        // драйвер 4 пин полушаг
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4, en);    // драйвер 4 пин полушаг + enable

GStepper<STEPPER2WIRE, STEPPER_VIRTUAL> stepper(steps);                    // виртуальный драйвер step-dir
GStepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper(steps);                    // виртуальный драйвер 4 пин

// Здесь происходит движение мотора, вызывать как можно чаще!
// Имеет встроенный таймер
// Возвращает true, если мотор движется к цели или крутится по KEEP_SPEED
bool tick();

// Инвертировать направление мотора - true (по умолч. false)
void reverse(bool dir);

// инвертировать поведение EN пина - true (по умолч. false)
void invertEn(bool rev);

// Установка режима работы, mode:
// FOLLOW_POS - следование к позиции setTarget(...)
// KEEP_SPEED - удержание скорости setSpeed(...)
void setRunMode(GS_runMode mode);

// Установка текущей позиции мотора в шагах и градусах
void setCurrent(int32_t pos);
void setCurrentDeg(float pos);

// Чтение текущей позиции мотора в шагах и градусах
int32_t getCurrent();
float getCurrentDeg();

// установка целевой позиции в шагах и градусах (для режима FOLLOW_POS)
// type - ABSOLUTE или RELATIVE, по умолчанию стоит ABSOLUTE
// RELATIVE считается от текущей позиции мотора
void setTarget(int32_t pos);
void setTarget(int32_t pos, GS_posType type);
void setTargetDeg(float pos);
void setTargetDeg(float pos, GS_posType type);

// Получение целевой позиции в шагах и градусах
int32_t getTarget();
float getTargetDeg();

// Установка максимальной скорости (по модулю) в шагах/секунду и градусах/секунду (для режима FOLLOW_POS)
// по умолч. 300
// минимум - 1 шаг в час
void setMaxSpeed(float speed);
void setMaxSpeedDeg(float speed);

// Установка ускорения в шагах и градусах в секунду (для режима FOLLOW_POS).
// При значении 0 ускорение отключается и мотор работает
// по профилю постоянной максимальной скорости setMaxSpeed().
// По умолч. 300
void setAcceleration(int accel);
void setAccelerationDeg(float accel);

// Автоотключение EN при достижении позиции - true (по умолч. false).
void autoPower(bool mode);

// Плавная остановка с заданным ускорением от текущего положения
// Работает также в режиме KEEP_SPEED
void stop();

// Жёсткая остановка. Отключает мотор, если включен autoPower
void brake();

// Жёсткая остановка + сброс позиции в 0 (для концевиков)
void reset();

// Установка целевой скорости в шагах/секунду и градусах/секунду (для режима KEEP_SPEED)
// при ненулевом setAcceleration будет выполнен плавный разгон/торможение к нужной скорости
// минимальная скорость - 1 шаг в час
void setSpeed(float speed);
void setSpeedDeg(float speed);

// Получение целевой скорости в шагах/секунду и градусах/секунду (для режима KEEP_SPEED)
float getSpeed();
float getSpeedDeg();

// Включить мотор (пин EN)
void enable();

// Выключить мотор (пин EN)
void disable();

// Возвращает то же самое, что tick, т.е. крутится мотор или нет
bool getState();

// Возвращает минимальный период тика мотора в микросекундах при настроенной setMaxSpeed() скорости.
// Можно использовать для настройки прерываний таймера, в обработчике которого будет лежать tick() (см. пример timerISR)
uint32_t getMinPeriod();

// Текущий период "тика" для отладки и всего такого
uint32_t stepTime;

// подключить внешний обработчик для шага и переключения питания
void attachStep(handler)
void attachPower(handler)

*/

// Раскомментируй для использования более плавного, но медленного алгоритма
// Также дефайн можно прописать в скетче до подключения библиотеки!!! См. пример smoothAlgorithm
// #define SMOOTH_ALGORITHM

#ifndef _GyverStepper_h
#define _GyverStepper_h
#include <Arduino.h>

#include "GStypes.h"
#include "StepperCore.h"

// =========== МАКРОСЫ ===========
#define degPerMinute(x) ((x) / 60.0f)
#define degPerHour(x) ((x) / 3600.0f)
#define _sign(x) ((x) >= 0 ? 1 : -1)  // знак числа

// =========== КОНСТАНТЫ ===========
#define _MIN_SPEED_FP 5  // мин. скорость для движения в FOLLOW_POS с ускорением
#define _MAX_PERIOD_FP (1000000L / _MIN_SPEED_FP)
#define _MIN_STEP_SPEED (1.0 / 3600)  // мин. скорость 1 шаг в час

enum GS_runMode {
    FOLLOW_POS,
    KEEP_SPEED,
};

enum GS_smoothType {
    NO_SMOOTH,
    SMOOTH,
};

// ============================== GS CLASS ==============================
template <GS_driverType _DRV, GS_driverType _TYPE = STEPPER_PINS>
class GStepper : public Stepper<_DRV, _TYPE> {
   public:
    // конструктор
    GStepper(int stepsPerRev, uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255, uint8_t pin4 = 255, uint8_t pin5 = 255) : Stepper<_DRV, _TYPE>(pin1, pin2, pin3, pin4, pin5) {
        // умолчания
        setMaxSpeed(300);
        setAcceleration(300);
        _stepsPerDeg = (stepsPerRev / 360.0);
    }

    // ============================== TICK ==============================
    // тикер, вызывать почаще. Возвращает true, если мотор всё ещё движется к цели
    bool tick() {
        if (_workState) {
            tickUs = micros();
#ifndef SMOOTH_ALGORITHM
            if (!_curMode && _accel != 0 && _maxSpeed >= _MIN_SPEED_FP) planner();  // планировщик скорости FOLLOW_POS быстрый
#endif

            if (_curMode && _accel != 0) smoothSpeedPlanner();  // планировщик скорости KEEP_SPEED

            if (stepTime && tickUs - _prevTime >= stepTime) {  // основной таймер степпера
                _prevTime = tickUs;

#ifdef SMOOTH_ALGORITHM
                // плавный планировщик вызывается каждый шаг. Проверка остановки
                if (!_curMode && _accel != 0 && _maxSpeed >= _MIN_SPEED_FP && !plannerSmooth()) {
                    brake();
                    return false;
                }
#endif

                // проверка остановки для быстрого планировщика, а также работы без ускорения
                if (!_curMode && _target == pos) {
                    brake();
                    return false;
                }
                step();  // двигаем мотор
            }
        }
        return _workState;
    }

    // ============================== SETTINGS ==============================

    // установка текущей позиции в шагах
    void setCurrent(int32_t npos) {
        pos = npos;
        _accelSpeed = 0;
    }

    // установка текущей позиции в градусах
    void setCurrentDeg(float npos) {
        setCurrent((float)npos * _stepsPerDeg);
    }

    // чтение текущей позиции в шагах
    int32_t getCurrent() {
        return pos;
    }

    // чтение текущей позиции в градусах
    float getCurrentDeg() {
        return ((float)pos / _stepsPerDeg);
    }

    // установка целевой позиции в шагах
    void setTarget(int32_t npos, GS_posType type = ABSOLUTE) {
        _target = type ? (npos + pos) : npos;
        if (_target != pos) {
            if (_accel == 0 || _maxSpeed < _MIN_SPEED_FP) {
                stepTime = 1000000.0 / _maxSpeed;
                dir = (_target > pos) ? 1 : -1;
            }
            enable();
        }
    }

    // установка целевой позиции в градусах
    void setTargetDeg(double npos, GS_posType type = ABSOLUTE) {
        setTarget((float)npos * _stepsPerDeg, type);
    }

    // получение целевой позиции в шагах
    int32_t getTarget() {
        return _target;
    }

    // целевой позиции в градусах
    float getTargetDeg() {
        return ((float)_target / _stepsPerDeg);
    }

    // установка максимальной скорости в шагах/секунду
    void setMaxSpeed(double speed) {
        speed = abs(speed);
        _maxSpeed = max(speed, _MIN_STEP_SPEED);  // 1 шаг в час минимум
        // считаем stepTime для низких скоростей или отключенного ускорения
        if (_accel == 0 || _maxSpeed < _MIN_SPEED_FP) stepTime = 1000000.0 / _maxSpeed;

#ifdef SMOOTH_ALGORITHM
        _cmin = 1000000.0 / _maxSpeed;
        if (_n > 0) {
            _n = (float)_accelSpeed * _accelSpeed * _accelInv;
            plannerSmooth();
        }
#else
        // период планировщка в зависимости от макс. скорости
        _plannerPrd = map((int)_maxSpeed, 1000, 20000, 15000, 1000);
        _plannerPrd = constrain(_plannerPrd, 15000, 1000);
#endif
    }

    // установка максимальной скорости в градусах/секунду
    void setMaxSpeedDeg(double speed) {
        setMaxSpeed(speed * _stepsPerDeg);
    }

    // установка ускорения в шагах/секунду^2
    void setAcceleration(uint16_t accel) {
        _accel = accel;
        if (_accel) _accelInv = 0.5f / accel;
        else _accelInv = 0;
        _accelTime = accel / 1000000.0f;
#ifdef SMOOTH_ALGORITHM
        if (_accel) _c0 = 0.676 * sqrt(2.0 / _accel) * 1000000.0;
        plannerSmooth();
#endif
    }

    // установка ускорения в градусах/секунду^2
    void setAccelerationDeg(float accel) {
        setAcceleration(accel * _stepsPerDeg);
    }

    // автоотключение питания при остановке
    void autoPower(bool mode) {
        _autoPower = mode;
    }

    // плавная остановка с заданным ускорением
    void stop() {
        if (_workState) {
            resetTimers();
            if (_curMode == FOLLOW_POS) {
                if (!_accel) {
                    brake();
                    return;
                }
                _accelSpeed = 1000000.0f / stepTime * dir;
                setTarget(pos + (float)_accelSpeed * _accelSpeed * _accelInv * dir);
                // setMaxSpeed(abs(_accelSpeed));
                _stopSpeed = abs(_accelSpeed);
#ifdef SMOOTH_ALGORITHM
                _n = (float)_accelSpeed * _accelSpeed * _accelInv;
#endif
            } else {
                setSpeed(0);
            }
        }
    }

    // остановка и сброс позиции в 0
    void reset() {
        brake();
        setCurrent(0);
    }

    // установка целевой скорости в шагах/секунду
    void setSpeed(float speed, bool smooth = false) {  // smooth убран!
        // 1 шаг в час минимум
        _speed = speed;
        _stopF = (_speed == 0);
        if (_speed == 0 && _accelSpeed == 0) return;
        dir = (_speed > 0) ? 1 : -1;
        if (abs(_speed) < _MIN_STEP_SPEED) _speed = _MIN_STEP_SPEED * dir;

        if (_accel != 0) {  // плавный старт
            if (_accelSpeed != _speed) {
                int speed1 = (int)abs(_speed);
                int speed2 = (int)abs(_accelSpeed);
                _speedPlannerPrd = map(max(speed1, speed2), 1000, 20000, 15000, 2000);
                _speedPlannerPrd = constrain(_speedPlannerPrd, 15000, 2000);
                stepTime = abs(1000000.0 / _accelSpeed);
            }
        } else {               // резкий старт
            if (speed == 0) {  // скорость 0? Отключаемся и выходим
                brake();
                return;
            }
            _accelSpeed = _speed;
            stepTime = abs(1000000.0 / _speed);
        }
        enable();
    }

    // установка целевой скорости в градусах/секунду
    void setSpeedDeg(float speed, bool smooth = false) {
        setSpeed(_stepsPerDeg * speed);
    }

    // получение целевой скорости в шагах/секунду
    float getSpeed() {
        return (1000000.0 / stepTime * dir);
    }

    // получение целевой скорости в градусах/секунду
    float getSpeedDeg() {
        return ((float)getSpeed() / _stepsPerDeg);
    }

    // установка режима работы
    void setRunMode(GS_runMode mode) {
        if (_curMode != mode) resetTimers();
        _curMode = mode;
    }

    // получить статус вкл/выкл
    bool getState() {
        return _workState;
    }

    // включить мотор
    void enable() {
        _workState = true;
        _stopSpeed = 0;
        resetTimers();
        Stepper<_DRV, _TYPE>::enable();
    }

    // резкая остановка
    void brake() {
        _workState = false;
        _stopSpeed = 0;
        resetMotor();
        if (_autoPower) Stepper<_DRV, _TYPE>::disable();
    }

    // получить минимальный период, с которым нужно вызывать tick при заданной макс. скорости
    uint32_t getMinPeriod() {
        float curSpeed;
        if (_curMode == KEEP_SPEED) {
            curSpeed = abs(_speed);
            if (abs(_accelSpeed) > curSpeed) curSpeed = abs(_accelSpeed);
        } else curSpeed = _maxSpeed;
        return (1000000.0 / curSpeed);
    }

    // время между шагами
    uint32_t stepTime = 10000;

    using Stepper<_DRV, _TYPE>::pos;
    using Stepper<_DRV, _TYPE>::dir;
    using Stepper<_DRV, _TYPE>::step;
    using Stepper<_DRV, _TYPE>::enable;
    using Stepper<_DRV, _TYPE>::disable;

    // ========================= PRIVATE ==========================
   private:
    // сброс перемещения
    void resetMotor() {
        _accelSpeed = 0;
#ifdef SMOOTH_ALGORITHM
        _n = 0;
#endif
    }
    // аккуратно сбросить все таймеры
    void resetTimers() {
        _speedPlannerTime = _plannerTime = _prevTime = micros();
    }

#ifdef SMOOTH_ALGORITHM
    // ========================= PLANNER 1 =========================
    // планировщик скорости из AccelStepper
    bool plannerSmooth() {
        int32_t err = _target - pos;
        int32_t stepsToStop = (float)_accelSpeed * _accelSpeed * _accelInv;

        if (err == 0 && stepsToStop <= 1) return false;

        if (err > 0) {
            if (_n > 0) {
                if ((stepsToStop >= err) || dir == -1)
                    _n = -stepsToStop;
            } else if (_n < 0) {
                if ((stepsToStop < err) && dir == 1)
                    _n = -_n;
            }
        } else if (err < 0) {
            if (_n > 0) {
                if ((stepsToStop >= -err) || dir == 1)
                    _n = -stepsToStop;
            } else if (_n < 0) {
                if ((stepsToStop < -err) && dir == -1)
                    _n = -_n;
            }
        }

        if (_n == 0) {
            _cn = _c0;
            dir = _sign(err);
        } else {
            _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1));
            _cn = max(_cn, _cmin);
        }
        _n++;
        stepTime = _cn;
        _accelSpeed = 1000000.0 / _cn;
        if (dir == -1) _accelSpeed = -_accelSpeed;
        return true;
    }

    int32_t _n = 0;
    float _c0 = 0.0;
    float _cn = 0.0;
    float _cmin = 1.0;
#else
    // ========================= PLANNER 2 =========================
    // планировщик скорости мой
    void planner() {
        if (tickUs - _plannerTime >= _plannerPrd) {
            _plannerTime += _plannerPrd;
            // ~110 us
            int32_t err = _target - pos;                                                                  // "ошибка"
            bool thisDir = (_accelSpeed * _accelSpeed * _accelInv >= abs(err));                        // пора тормозить
            _accelSpeed += (_accelTime * _plannerPrd * (thisDir ? -_sign(_accelSpeed) : _sign(err)));  // разгон/торможение
            if (_stopSpeed == 0) _accelSpeed = constrain(_accelSpeed, -_maxSpeed, _maxSpeed);          // ограничение
            else _accelSpeed = constrain(_accelSpeed, -_stopSpeed, _stopSpeed);

            if (abs(_accelSpeed) > _MIN_SPEED_FP) stepTime = abs(1000000.0 / _accelSpeed);  // ограничение на мин. скорость
            else stepTime = _MAX_PERIOD_FP;
            dir = _sign(_accelSpeed);  // направление для шагов
        }
    }

    uint16_t _plannerPrd = 15000;
#endif

    // ======================= SPEED PLANNER =======================
    float _accelTime = 0;
    uint16_t _speedPlannerPrd = 15000;
    uint32_t _speedPlannerTime = 0;
    uint32_t _plannerTime = 0;

    // планировщик разгона для KEEP_SPEED
    void smoothSpeedPlanner() {
        if (tickUs - _speedPlannerTime >= _speedPlannerPrd) {
            _speedPlannerTime = tickUs;
            _accelSpeed += (_accelTime * _speedPlannerPrd * _sign(_speed - _accelSpeed));
            dir = _sign(_accelSpeed);
            stepTime = abs(1000000.0 / _accelSpeed);
            if (_stopF && abs(_accelSpeed) <= _MIN_STEP_SPEED) brake();
        }
    }

    // ========================= VARIABLES =========================
    bool _stopF = 0;
    float _stepsPerDeg;
    uint32_t _prevTime = 0;
    float _accelSpeed = 0;
    int32_t _target = 0;
    volatile uint32_t tickUs = 0;
    bool _workState = false;
    bool _autoPower = false;
    float _stopSpeed = 0;
    float _maxSpeed = 300;
    float _speed = 0;
    uint16_t _accel = 0;
    float _accelInv = 0;
    GS_runMode _curMode = FOLLOW_POS;
};
#endif
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
*/

/*
// Примечание: далее по тексту под "по умолчанию" имеется в виду "даже если не вызывать функцию"

// Создание объекта
// steps - шагов на один оборот вала (для расчётов с градусами)
// step, dir, pin1, pin2, pin3, pin4 - любые GPIO
// en - пин отключения драйвера, любой GPIO
GStepper<STEPPER2WIRE> stepper(steps, step, dir);						// драйвер step-dir
GStepper<STEPPER2WIRE> stepper(steps, step, dir, en);					// драйвер step-dir + пин enable
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4);			// драйвер 4 пин
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4, en);		// драйвер 4 пин + enable
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4);		// драйвер 4 пин полушаг
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4, en);	// драйвер 4 пин полушаг + enable

GStepper<STEPPER2WIRE, STEPPER_VIRTUAL> stepper(steps);					// виртуальный драйвер step-dir
GStepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper(steps);					// виртуальный драйвер 4 пин

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
void setCurrent(long pos);
void setCurrentDeg(float pos);

// Чтение текущей позиции мотора в шагах и градусах
long getCurrent();
float getCurrentDeg();

// установка целевой позиции в шагах и градусах (для режима FOLLOW_POS)
// type - ABSOLUTE или RELATIVE, по умолчанию стоит ABSOLUTE
// RELATIVE считается от текущей позиции мотора
void setTarget(long pos);
void setTarget(long pos, GS_posType type);
void setTargetDeg(float pos);
void setTargetDeg(float pos, GS_posType type);

// Получение целевой позиции в шагах и градусах
long getTarget();
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
//#define SMOOTH_ALGORITHM

#ifndef _GyverStepper_h
#define _GyverStepper_h
#include <Arduino.h>
#include "GStypes.h"
#include "StepperCore.h"

// =========== МАКРОСЫ ===========
#define degPerMinute(x) ((x)/60.0f)
#define degPerHour(x) ((x)/3600.0f)
#define _sign(x) ((x) >= 0 ? 1 : -1)	// знак числа
#define maxMacro(a,b) ((a)>(b)?(a):(b))	// привет esp

// =========== КОНСТАНТЫ ===========
#define _MIN_SPEED_FP 5		// мин. скорость для движения в FOLLOW_POS с ускорением
#define _MAX_PERIOD_FP (1000000L/_MIN_SPEED_FP)
#define _MIN_STEP_SPEED (1.0f/3600)	// мин. скорость 1 шаг в час

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
    GStepper(int stepsPerRev, uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255, uint8_t pin4 = 255, uint8_t pin5 = 255) : 
    _stepsPerDeg(stepsPerRev / 360.0) {
        setPins(pin1, pin2, pin3, pin4, pin5);
        // умолчания
        setMaxSpeed(300);
        setAcceleration(300);
    }
    
    // ============================== TICK ==============================
    // тикер, вызывать почаще. Возвращает true, если мотор всё ещё движется к цели
    bool tick() {
        if (_workState) {
            tickUs = micros();
            #ifndef SMOOTH_ALGORITHM				
            if (!_curMode && _accel != 0 && _maxSpeed >= _MIN_SPEED_FP) planner();	// планировщик скорости FOLLOW_POS быстрый		
            #endif
            
            if (_smoothStart && _curMode) smoothSpeedPlanner();		// планировщик скорости KEEP_SPEED
            
            if (tickUs - _prevTime >= stepTime) {					// основной таймер степпера
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
                step();     // двигаем мотор
            }
        }
        return _workState;
    }    
    
    // ============================== SETTINGS ==============================

    // установка текущей позиции в шагах
    void setCurrent(long npos) {
        pos = npos;
        _accelSpeed = 0;
    }
    
    // установка текущей позиции в градусах
    void setCurrentDeg(float pos) {
        setCurrent((float)pos * _stepsPerDeg);
    }
    
    // чтение текущей позиции в шагах
    long getCurrent() {
        return pos;
    }
    
    // чтение текущей позиции в градусах
    float getCurrentDeg() {
        return ((float)pos / _stepsPerDeg);
    }

    // установка целевой позиции в шагах
    void setTarget(long pos, GS_posType type = ABSOLUTE) {
        _target = type ? (pos + pos) : pos;		
        if (_target != pos) {
            if (_accel == 0 || _maxSpeed < _MIN_SPEED_FP) {
                stepTime = 1000000.0 / _maxSpeed;
                dir = (_target > pos) ? 1 : -1;
            }
            enable();
        }
    }
    
    // установка целевой позиции в градусах
    void setTargetDeg(float pos, GS_posType type = ABSOLUTE) {
        setTarget((float)pos * _stepsPerDeg, type);
    }
    
    // получение целевой позиции в шагах
    long getTarget() {
        return _target;
    }
    
    // целевой позиции в градусах
    float getTargetDeg() {
        return ((float)_target / _stepsPerDeg);
    }

    // установка максимальной скорости в шагах/секунду
    void setMaxSpeed(float speed) {
        speed = abs(speed);
        _maxSpeed = maxMacro(speed, _MIN_STEP_SPEED);	// 1 шаг в час минимум
        // считаем stepTime для низких скоростей или отключенного ускорения
        if (_accel == 0 || _maxSpeed < _MIN_SPEED_FP) stepTime = 1000000.0 / _maxSpeed;
        
        #ifdef SMOOTH_ALGORITHM
        _cmin = 1000000.0 / _maxSpeed;
        if (_n > 0)	{
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
    void setMaxSpeedDeg(float speed) {
        setMaxSpeed((float)speed * _stepsPerDeg);
    }

    // установка ускорения в шагах/секунду^2
    void setAcceleration(int accel)	{
        _accel = abs(accel);
        _accelInv = 0.5f / accel;
        _accelTime = accel / 1000000.0f;
        #ifdef SMOOTH_ALGORITHM
        _n = _n * (_accel / accel);
        _c0 = 0.676 * sqrt(2.0 / _accel) * 1000000.0;
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
                _accelSpeed = 1000000.0f / stepTime * dir;
                setTarget(pos + (float)_accelSpeed * _accelSpeed * _accelInv * dir);
                //setMaxSpeed(abs(_accelSpeed));
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
    void setSpeed(float speed, bool smooth = false) {	// smooth убран!
        // 1 шаг в час минимум
        _speed = speed;
        if (abs(_speed) < _MIN_STEP_SPEED) _speed = _MIN_STEP_SPEED * _sign(_speed);
        enable();
        if (_accel != 0) {							// плавный старт		
            if (_accelSpeed != _speed) {
                _smoothStart = true;
                #ifdef __AVR__
                _smoothPlannerPrd = map(max(abs((int)_speed), abs((int)_accelSpeed)), 1000, 20000, 15000, 1000);
                #else
                // горячий привет тупому компилятору ESP8266 и индусам, которые его настраивали
                int speed1 = abs(_speed);
                int speed2 = abs((int)_accelSpeed);
                int maxSpeed = maxMacro(speed1, speed2);
                _smoothPlannerPrd = map(maxSpeed, 1000, 20000, 15000, 1000);
                #endif			
                _smoothPlannerPrd = constrain(_smoothPlannerPrd, 15000, 1000);
            }
        } else {				// резкий старт
            if (speed == 0) { 	// скорость 0? Отключаемся и выходим
                brake();
                return;
            }	
            _accelSpeed = _speed;
            stepTime = round(1000000.0 / abs(_speed));
            dir = (_speed > 0) ? 1 : -1;
        }        
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
    void setRunMode(GS_runMode mode){
        _curMode = mode; 
        if (mode == FOLLOW_POS) _smoothStart = false;
        resetTimers();
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
        if (_autoPower) disable();        
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
    
    // ========================= PRIVATE ==========================
private:
    using Stepper<_DRV, _TYPE>::pos;
    using Stepper<_DRV, _TYPE>::dir;
    using Stepper<_DRV, _TYPE>::setPins;
    using Stepper<_DRV, _TYPE>::step;
    using Stepper<_DRV, _TYPE>::enable;
    using Stepper<_DRV, _TYPE>::disable;
    
    // сброс перемещения
    void resetMotor() {
        _accelSpeed = 0;
        #ifdef SMOOTH_ALGORITHM
        _n = 0;
        #endif	
    }
    // аккуратно сбросить все таймеры
    void resetTimers() {        
        _smoothPlannerTime = _plannerTime = _prevTime = micros();        
    }
    

#ifdef SMOOTH_ALGORITHM
    // ========================= PLANNER 1 =========================
    // планировщик скорости из AccelStepper
    bool plannerSmooth() {		
        long err = _target - pos;
        long stepsToStop = (float)_accelSpeed * _accelSpeed * _accelInv;
        
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
    
    long _n = 0;
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
            long err = _target - pos;											// "ошибка"
            bool thisDir = ( _accelSpeed * _accelSpeed * _accelInv >= abs(err) );	// пора тормозить
            _accelSpeed += ( _accelTime * _plannerPrd * (thisDir ? -_sign(_accelSpeed) : _sign(err)) );	// разгон/торможение
            if (_stopSpeed == 0) _accelSpeed = constrain(_accelSpeed, -_maxSpeed, _maxSpeed);   // ограничение
            else _accelSpeed = constrain(_accelSpeed, -_stopSpeed, _stopSpeed);

            if (abs(_accelSpeed) > _MIN_SPEED_FP) stepTime = abs(1000000.0 / _accelSpeed);		// ограничение на мин. скорость
            else stepTime = _MAX_PERIOD_FP;
            dir = _sign(_accelSpeed);												// направление для шагов
        }
    }

    int _plannerPrd = 15000;
#endif

    // ======================= SPEED PLANNER =======================
    float _accelTime = 0;
    int _smoothPlannerPrd = 15000;
    uint32_t _smoothPlannerTime = 0;
    uint32_t _plannerTime = 0;
    
    // планировщик разгона для KEEP_SPEED
    void smoothSpeedPlanner() {
        if (tickUs - _smoothPlannerTime >= _smoothPlannerPrd) {
            _smoothPlannerTime = tickUs;
            int8_t dir = _sign(_speed - _accelSpeed);	// 1 - разгон, -1 - торможение
            _accelSpeed += (_accelTime * _smoothPlannerPrd * dir);			
            dir = _sign(_accelSpeed);
            
            // прекращение работы планировщика
            if ((dir == 1 && _accelSpeed >= _speed) || (dir == -1 && _accelSpeed <= _speed)) {
                _accelSpeed = _speed;
                _smoothStart = false;
                if (abs(_speed) <= _MIN_STEP_SPEED) {		// если нужно остановиться
                    brake();
                    return;		// выходим
                }
            }
            stepTime = abs(1000000.0 / _accelSpeed);
        }
    }
    
    // ========================= VARIABLES =========================
    const float _stepsPerDeg;
    uint32_t _prevTime = 0;
    float _accelSpeed = 0;
    int32_t _target = 0;
    volatile uint32_t tickUs = 0;
    bool _workState = false;
    bool _autoPower = false;
    bool _smoothStart = false;
    float _stopSpeed = 0;
    float _maxSpeed = 300;
    float _speed = 0;
    int _accel = 0;	
    float _accelInv = 0;
    GS_runMode _curMode = FOLLOW_POS;
};
#endif
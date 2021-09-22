/*
    Облегчённая GyverStepper
    - Легче на несколько кБ, всё целочисленное
    - Более эффективный гибридный алгоритм
    - Движение к цели с ускорением
    - Движение от точки к точке. Смена точки во время движения не будет плавной
    - Вращение со скоростью (без плавной смены скорости)
    - Оптимизировано для работы по прерыванию таймера
    - Наследует класс Stepper из StepperCore

    Версии:
    v1.0
*/

/*
    // ======== ИНИЦИАЛИЗАЦИЯ как в GStepper ========
    GStepper2<STEPPER2WIRE> stepper(шаговНаОборот, step, dir);						 // драйвер step-dir
    GStepper2<STEPPER2WIRE> stepper(шаговНаОборот, step, dir, en);					 // драйвер step-dir + пин enable
    GStepper2<STEPPER4WIRE> stepper(шаговНаОборот, pin1, pin2, pin3, pin4);			 // драйвер 4 пин
    GStepper2<STEPPER4WIRE> stepper(шаговНаОборот, pin1, pin2, pin3, pin4, en);		 // драйвер 4 пин + enable
    GStepper2<STEPPER4WIRE_HALF> stepper(шаговНаОборот, pin1, pin2, pin3, pin4);     // драйвер 4 пин полушаг
    GStepper2<STEPPER4WIRE_HALF> stepper(шаговНаОборот, pin1, pin2, pin3, pin4, en); // драйвер 4 пин полушаг + enable

    GStepper2<STEPPER2WIRE, STEPPER_VIRTUAL> stepper;    // виртуальный драйвер step-dir
    GStepper2<STEPPER4WIRE, STEPPER_VIRTUAL> stepper;    // виртуальный драйвер 4 пин

    // ============ КЛАСС ============
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
    void setSpeedDeg(int16_t speed);            // установить скорость в градусах/сек и запустить вращение
    void setSpeedDeg(float speed);              // установить скорость в градусах/сек (float) и запустить вращение
    
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
*/

#ifndef _GyverStepper2_h
#define _GyverStepper2_h
#include <Arduino.h>
#include "StepperCore.h"
#define GS_MIN_US 300000     // период, длиннее которого мотор можно резко тормозить или менять скорость

template <GS_driverType _DRV, GS_driverType _TYPE = STEPPER_PINS>
class GStepper2 : public Stepper<_DRV, _TYPE> {
public:
    // ========================= КОНСТРУКТОР ==========================
    GStepper2(uint16_t steps, uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255, uint8_t pin4 = 255, uint8_t pin5 = 255) {
        setPins(pin1, pin2, pin3, pin4, pin5);
        stepsRev = steps;
        setMaxSpeed(100);
    }
    
    // ============================= TICK =============================
    // тикер. Вернёт true, если мотор движется
    bool tick() {
        if (status) {
            uint32_t thisUs = micros();
            if (thisUs - tmr >= us) {
                tmr = thisUs;
                tickManual();
            }
        }
        return status;
    }
    
    // ручной тикер для вызова в прерывании таймера. Вернёт true, если мотор движется
    bool tickManual() {
        if (!status) return 0;  // стоим-выходим
        step();                 // шаг
        
    #ifndef GS_NO_ACCEL     // движение с ускорением
        switch (status) {
        case 1:     // едем
        case 2:     // пауза
            // https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
            steps++;
            if (steps < s1) {                                    // разгон
                us10 -= 2ul * us10 / (4ul * (steps) + 1);
                us = (uint32_t)us10 >> 10;
                us = constrain(us, usMin, us0);
            }
            else if (steps < s2) us = usMin;                     // постоянная
            else if (steps < S) {                                // торможение
                us10 += 2ul * us10 / (4ul * (S - steps) + 1);
                us = (uint32_t)us10 >> 10;
                us = constrain(us, usMin, us0);
            } else {                                             // приехали
                if (status == 1) readyF = 1;
                brake();
            }
            return status;
        case 4:     // плавная остановка
            stopStep--;
            us10 += 2ul * us10 / (4ul * stopStep + 1);
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
            if (pos == tar || stopStep <= 0 || us >= us0) brake();
            return status;
        }
    #else
        if (status <= 2 && pos == tar) {
            if (status == 1) readyF = 1;
            brake();
        }
    #endif
        return status;
    }

    // ============================= SPEED MODE =============================
    // установить скорость вращения
    bool setSpeed(int32_t speed) {
        if (speed == 0) {
            brake();
            return 0;
        }
        dir = (speed > 0) ? 1 : -1;
        us = 1000000L / abs(speed);
        status = 3;
        return 1;
    }
    
    #ifndef ESP8266
    void setSpeed(int speed) {
        setSpeed((int32_t)speed);
    }
    #endif
    
    // установить скорость вращения float
    void setSpeed(double speed) {
        if (setSpeed((int32_t)speed)) us = 1000000.0 / abs(speed);
    }
    
    void setSpeedDeg(int speed) {
        setSpeed((int32_t)speed * stepsRev / 360L);
    }
    
    void setSpeedDeg(double speed) {
        setSpeed((float)speed * stepsRev / 360L);
    }
    
    // =========================== POSITION MODE ===========================
    // установить цель и опционально режим
    void setTarget(int32_t ntar, GS_posType type = ABSOLUTE) {
        tmr = micros();
        if (type == RELATIVE) tar = ntar + pos;
        else tar = ntar;
        if (tar == pos) {
            brake();
            readyF = 1;
            return;
        }
        dir = (pos < tar) ? 1 : -1;
        
    #ifndef GS_NO_ACCEL
        S = abs(pos - tar);
        
        // расчёт точек смены характера движения
        // s1 - окончание разгона, s1-s2 - равномерное движение, s2 - торможение
        if (a > 0 && usMin < GS_MIN_US) {           // ускорение задано и мин. скорость выше порога
            if ((int32_t)V * V > (int32_t)a * S) {  // треугольник
                s1 = S / 2L;
                s2 = s1;
            } else {                                // трапеция
                s1 = (int32_t)V * V / (2 * a);
                s2 = S - s1;
            }
            us = us0;
        } else {
            s1 = 0;
            s2 = S;
            us = usMin;
        }
        // здесь us10 - us*1024 для повышения разрешения микросекунд в 1024 раз
        us10 = (uint32_t)us << 10;
        steps = 0;
    #else
        us = usMin;
    #endif
        status = 1;
        readyF = 0;
    }
    
    // установить цель в градусах и опционально режим
    void setTargetDeg(int32_t ntar, GS_posType type = ABSOLUTE) {
        setTarget((int32_t)ntar * stepsRev / 360L, type);
    }
    void setTargetDeg(int16_t ntar, GS_posType type = ABSOLUTE) {
        setTarget((int32_t)ntar * stepsRev / 360L, type);
    }
    
    // установить цель в градусах float и опционально режим
    void setTargetDeg(double ntar, GS_posType type = ABSOLUTE) {
        setTarget((float)ntar * stepsRev / 360.0, type);
    }
    
    // получить целевую позицию
    int32_t getTarget() {
        return tar;
    }
    
    // установить текущую позицию
    void setCurrent(int32_t npos) {
        pos = npos;
    }
    
    // получить текущую позицию
    int32_t getCurrent() {
        return pos;
    }
    
    // сбросить текущую позицию в 0
    void reset() {
        pos = 0;
    }

    // ========================== POSITION SETTINGS ==========================
    // установка ускорения в шаг/сек^2
    void setAcceleration(uint16_t nA) {
        #ifndef GS_NO_ACCEL
        a = nA;
        if (a != 0) us0 = 0.676 * 1000000 * sqrt(2.0 / a);
        else us0 = usMin;
        #endif
    }
    
    // установить скорость движения при следовании к позиции в шагах/сек
    void setMaxSpeed(int32_t speed) {
        usMin = 1000000L / speed;
        #ifndef GS_NO_ACCEL
        V = (uint16_t)speed;  // если < 1, отсечётся до 0
        #endif
    }
    
    #ifndef ESP8266
    void setMaxSpeed(int speed) {
        setMaxSpeed((int32_t)speed);
    }
    #endif
    
    // установить скорость движения при следовании к позиции в шагах/сек, float
    void setMaxSpeed(float speed) {
        usMin = 1000000.0 / speed;
        #ifndef GS_NO_ACCEL
        V = speed;  // если < 1, отсечётся до 0
        #endif
    }
    
    // установить скорость движения при следовании к позиции в град/сек
    void setMaxSpeedDeg(int32_t speed) {
        setMaxSpeed((int32_t)speed * stepsRev / 360);
    }
    
    // установить скорость движения при следовании к позиции в град/сек, float
    void setMaxSpeedDeg(float speed) {
        setMaxSpeed((float)speed * stepsRev / 360.0);
    }
    
    // =========================== PLANNER ============================
    // остановить плавно (с заданным ускорением)
    void stop() {
    #ifndef GS_NO_ACCEL
        if (us == 0 || a == 0 || us > GS_MIN_US || status == 3 || !status) {   // нет ускорения или медленно едем - дёргай ручник
            brake();
            return;
        }
        if (status <= 2) {      // едем
            if (steps > s2) {   // а мы уже тормозим!
                pause();        // значит флаг на паузу
                return;
            }
            status = 4;
            stopStep = 1000000ul / us;                              // наша скорость
            stopStep = (uint32_t)stopStep * stopStep / (2 * a);     // дистанция остановки
            us10 = (uint32_t)us << 10;
        }
    #else
        brake();
    #endif
    }
    
    // остановить мотор
    void brake() {
        status = 0;
    }
    
    // пауза (доехать до заданной точки и ждать). ready() не вернёт true, пока ты на паузе
    void pause() {
        if (status == 1) status = 2;
    }
    
    // продолжить движение после остановки
    void resume() {
        if (!status) setTarget(tar);    
    }
    
    // текущий статус: 0 - стоим, 1 - едем, 2 - едем к точке паузы, 3 - крутимся со скоростью, 4 - тормозим
    uint8_t getStatus() {
        return status;
    }

    // вернёт true, если мотор доехал до установленной позиции и остановился
    bool ready() {
        if (!status && readyF) {
            readyF = 0;
            return 1;
        } return 0;
    }
    
    // получить текущий период вращения
    uint32_t getPeriod() {
        return us;
    }
    
    // чики пуки
    using Stepper<_DRV, _TYPE>::pos;
    using Stepper<_DRV, _TYPE>::dir;
    using Stepper<_DRV, _TYPE>::setPins;
    using Stepper<_DRV, _TYPE>::step;

// ============================= PRIVATE =============================
private:
    uint32_t tmr = 0, us = 10000, usMin = 10000;
    int32_t tar = 0;
    uint16_t stepsRev;
    uint8_t status = 0;
    bool readyF = 0;
    
    #ifndef GS_NO_ACCEL
    uint16_t a, V;
    int16_t stopStep;
    uint32_t S, us0, us10;
    int32_t s1, s2, steps;
    #endif
};
#endif
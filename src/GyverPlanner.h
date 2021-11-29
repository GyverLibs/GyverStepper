/*
    Многоосевой планировщик траекторий для шаговых моторов
    - ОСТАНОВКА В КАЖДОЙ ТОЧКЕ. БУФЕР НА ОДНУ СЛЕДУЮЩУЮ ПОЗИЦИЮ
    - Макс. скорость: 
      - Обычный режим: 37000 шаг/с на полной, 14000 шаг/с на разгоне
      - Быстрый профиль: 37000 шаг/с на полной, 37000 шаг/с на разгоне
    - Трапецеидальный профиль скорости (планировщик 2-го порядка)
    - Настройка скорости и ускорения
    - Любое количество осей. Будут двигаться синхронно к заданным целям
    - Быстрая целочисленная модель планирования траектории и скорости    
    - Режим постоянного вращения для одной оси (для движения к концевику например)
    - Тормоз/плавная остановка/пауза на траектории планировщика
    - Оптимизировано для работы по прерыванию таймера
    - Быстрый контроль пинов шаговика для Arduino AVR    
    
    AlexGyver, alex@alexgyver.ru
    https://alexgyver.ru/
    MIT License
*/

/*
    GPlanner<драйвер, количество осей> planner;     // объявяление

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
    uint8_t getStatus();                        // текущий статус: 0 - стоим, 1 - едем, 2 - едем к точке паузы, 3 -крутимся со скоростью, 4 - тормозим

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

    // ОСОБЕННОСТИ
    - Планировщик не поддерживает горячую смену цели с плавным изменением скорости
*/

#ifndef _GyverPlanner_h
#define _GyverPlanner_h
#include <Arduino.h>
#include "StepperCore.h"

#define GP_MIN_US 300000     // период, длиннее которого мотор можно резко тормозить или менять скорость

// создать планировщик с драйверами типа DRV и количеством осей AXLES
template < GS_driverType _DRV, uint8_t _AXLES >
class GPlanner {
public:
    GPlanner() {
        setAcceleration(100);
        setMaxSpeed(300);
    }
    
    // ============================== MOTOR ==============================
    // добавить объект типа Stepper на ось axis, начиная с 0
    void addStepper(uint8_t axis, Stepper<_DRV> &stp) {
        if (axis < _AXLES) steppers[axis] = &stp;
    }

    // ============================= PLANNER =============================        
    // true - готов принять следующую точку маршрута
    bool ready() {
        if (readyF && !status) {
            readyF = false;
            return true;
        } return false;
    }
    
    // установка максимальной скорости планировщика в шаг/сек
    void setMaxSpeed(float speed) {
        nV = speed;
        if (!status) {
            V = nV;
            usMin = 1000000.0 / V;
            setAcceleration(na);
            changeSett = 0;
        } else changeSett = 1;
    }
    
    // установка ускорения планировщика в шаг/сек^2
    void setAcceleration(uint16_t acc) {
        na = acc;
        if (!status) {
            a = na;
            if (a != 0) us0 = 0.676 * 1000000 * sqrt(2.0 / a);
            else us0 = usMin;
            changeSett = 0;
            calcPlan();
        } else changeSett = 1;
    }
    
    // пауза (доехать до заданной точки и ждать). ready() не вернёт true, пока ты на паузе
    void pause() {
        if (status == 1) status = 2;
    }

    // остановить плавно (с заданным ускорением)
    void stop() {
        if (us == 0 || a == 0 || us > GP_MIN_US || status == 3 || !status) {   // нет ускорения или крутим или медленно едем - дёргай ручник
            brake();
            return;
        }
        if (status <= 2) {      // едем
            if (step > s2) {    // а мы уже тормозим!
                pause();        // значит флаг на паузу
                return;
            }
            status = 4;
            stopStep = 1000000ul / us;                              // наша скорость
            stopStep = (uint32_t)stopStep * stopStep / (2 * a);     // дистанция остановки
            us10 = (uint32_t)us << 10;
        }
    }

    // резко остановить моторы из любого режима
    void brake() {
        status = 0;
        us = 0;
    }
    
    // продолжить после остановки/паузы
    void resume() {
        setTarget(tar);
    }

    // ============================= POSITION =============================
    // сбросить счётчики всех моторов в 0
    void reset() {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->pos = 0;
    }
    
    // установить текущее положение моторов
    void setCurrent(int16_t cur[]) {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->pos = (int32_t)cur[i];
    }
    void setCurrent(int32_t cur[]) {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->pos = cur[i];
    }
    
    // получить текущую позицию по оси axis
    int32_t getCurrent(int axis) {
        return steppers[axis]->pos;
    }
    
    // установить цель в шагах и начать движение. ~100 us
    bool setTarget(int32_t target[], GS_posType type = ABSOLUTE) {   
        if (changeSett) {       // применяем настройки
            setMaxSpeed(nV);
            changeSett = 0;
        }
        
        S = 0;                                              // путь
        for (uint8_t i = 0; i < _AXLES; i++) {              // для всех осей            
            if (type == RELATIVE) target[i] += steppers[i]->pos;    // если относительное смещение - прибавляем текущий pos
            tar[i] = target[i];                             // запоминаем цель
            dS[i] = abs(tar[i] - steppers[i]->pos);         // модуль ошибки по оси            
            steppers[i]->dir = (steppers[i]->pos < tar[i]) ? 1 : -1;  // направление движения по оси
            if (dS[i] > S) {                                // ищем максимальное отклонение
                S = dS[i];
                maxAx = i;                                  // запоминаем номер оси
            }
        }
        if (S == 0) {          // путь == 0, мы никуда не едем
            readyF = true;     // готовы к следующей точке
            brake();           // стоп машина
            return 0;
        }
        
        for (int i = 0; i < _AXLES; i++) nd[i] = S / 2u;    // записываем половину
        
        // расчёт точек смены характера движения
        // s1 - окончание разгона, s1-s2 - равномерное движение, s2 - торможение
        if (a > 0) {                                // ускорение задано
            if (us != 0 && us < GP_MIN_US) {        // мы движемся! ААА!
                int32_t v1 = 1000000L / us;
                if (2L * V * V - (int32_t)v1 * v1 > 2L * a * S) {  // треугольник
                    s1 = (2L * a * S - (int32_t)v1 * v1) / (4L * a);
                    s2 = s1;
                } else {                                // трапеция
                    s1 = ((int32_t)V * V - (int32_t)v1 * v1) / (2L * a);
                    s2 = S - (int32_t)V * V / (2 * a);
                }
                so1 = (int32_t)v1 * v1 / (2 * a);
            } else {                                    // не движемся
                if ((int32_t)V * V > (int32_t)a * S) {  // треугольник
                    s1 = S / 2L;
                    s2 = s1;
                } else {                                // трапеция
                    s1 = (int32_t)V * V / (2 * a);
                    s2 = S - s1;
                }
                so1 = 0;
                us = us0;
            }
        } else {        // ускорение отключено
            s1 = 0;
            s2 = S;
            us = usMin;
        }
        // здесь us10 - us*1024 для повышения разрешения микросекунд в 1024 раз
        us10 = (uint32_t)us << 10;
        step = 0;
        readyF = false;
        status = 1;        
        return 1;
    }
    
    bool setTarget(int16_t target[], GS_posType type = ABSOLUTE) {
        int32_t tar[_AXLES];
        for (uint8_t i = 0; i < _AXLES; i++) tar[i] = target[i];
        return setTarget(tar, type);
    }
    
    // получить цель в шагах на оси axis
    int32_t getTarget(int axis) {
        return tar[axis];
    }
    
    // ============================= SPEED ============================
    // режим постоянного вращения для оси axis со скоростью speed шаг/сек
    void setSpeed(uint8_t axis, float speed) {
        if (speed == 0) {    // это куда ты собрался?
            brake();
            return;
        }
        speedAxis = axis;                           // запомнили ось
        steppers[axis]->dir = speed > 0 ? 1 : -1;   // направление
        us = 1000000.0 / abs(speed);                // период
        status = 3;
    }
    
    // ============================= TICK =============================
    // тикер движения. Вернёт false если мотор остановлен. ~20..65us
    bool tick() {
        uint32_t now = micros();
        if (status > 0 && now - tmr >= us) {
            tmr = now;//+= us;//= micros();
            tickManual();
        }
        return status;
    }
    
    bool tickManual() {
        // режим постоянной скорости
        if (status == 3) {
            steppers[speedAxis]->step();
            return 1;
        }
        // здесь step - шаг вдоль общей линии траектории длиной S
        // шаги на проекциях получаются через алгоритм Брезенхема

        step++;
        for (uint8_t i = 0; i < _AXLES; i++) {
            // http://members.chello.at/easyfilter/bresenham.html
            if (i == maxAx) steppers[i]->step();    // этот движется всегда
            else {                                  // а эти по Брезенхему
                nd[i] -= dS[i];
                if (nd[i] < 0) {
                    nd[i] += S;
                    steppers[i]->step();
                }
            }
        }
        
        // плавная остановка
        if (status == 4) {
            stopStep--;
            us10 += 2ul * us10 / (4ul * stopStep + 1);    // торможение
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
            if (stopStep <= 0 || step >= S || us >= us0) brake();
            return status;
        }

        // https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
        // здесь us10 - us*1024 для повышения разрешения микросекунд
        if (step < s1) {                                // разгон
            #ifndef GS_FAST_PROFILE
            us10 -= 2ul * us10 / (4ul * (step + so1) + 1);
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
            #else
            if ((step + so1) >= prfS[GS_FAST_PROFILE - 1]) us = usMin;
            else {
                int j = 0;
                while ((step + so1) >= prfS[j]) j++;
                us = prfP[j];
            }
            #endif
        }
        else if (step < s2) us = usMin;                 // постоянная
        else if (step < S) {                            // торможение
            #ifndef GS_FAST_PROFILE
            us10 += 2ul * us10 / (4ul * (S - step) + 1);
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
            #else
            if ((S - step) >= prfS[GS_FAST_PROFILE - 1]) us = usMin;
            else {
                int j = 0;
                while ((S - step) >= prfS[j]) j++;
                us = prfP[j];
            }
            #endif
        } else {                                        // приехали
            if (status == 1) readyF = true;
            brake();
        }
        return status;
    }
    
    uint32_t getPeriod() {
        return us;
    }
    
    // текущий статус: 0 - стоим, 1 - едем, 2 - едем к точке паузы, 3 - крутимся со скоростью, 4 - тормозим
    uint8_t getStatus() {
        return status;
    }

    // ============================== PRIVATE ==============================
private:
    void calcPlan() {
        #ifdef GS_FAST_PROFILE
        if (a > 0) {
            uint32_t sa = (uint32_t)V * V / a / 2ul;            // расстояние разгона
            float dtf = sqrt(2.0 * sa / a) / GS_FAST_PROFILE;   // время участка профиля
            float s0 = a * dtf * dtf / 2.0;                     // первый участок профиля
            uint32_t dt = dtf * 1000000.0;                      // время участка в секундах
            for (int i = 0; i < GS_FAST_PROFILE; i++) {
                prfS[i] = s0 * (i + 1) * (i + 1);
                uint32_t ds = prfS[i];
                if (i > 0) ds -= prfS[i - 1];
                if (ds <= 0) prfP[i] = 0;
                else prfP[i] = (uint32_t)dt / ds;
            }
        }
        #endif
    }
    
    #ifdef GS_FAST_PROFILE
    uint32_t prfS[GS_FAST_PROFILE], prfP[GS_FAST_PROFILE];
    #endif
    
    uint32_t us;
    int32_t tar[_AXLES], nd[_AXLES], dS[_AXLES];
    int32_t step, S, s1, s2, so1;
    uint32_t tmr, us0, usMin, us10;
    uint16_t a, na;
    int16_t stopStep;
    float V, nV;    
    uint8_t status = 0, speedAxis = 0, maxAx;
    bool readyF = true;
    bool changeSett = 0;
    Stepper<_DRV>* steppers[_AXLES];
};
#endif
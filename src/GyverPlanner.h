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

    void setBacklash(uint8_t axis, uint16_t steps); // установить компенсацию люфта на ось axis в количестве шагов steps
    void enable();                                  // включить моторы
    void disable();                                 // выключить моторы
    void power(bool v);                             // переключить питание

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
    void home();                                // отправить в 0 по всем осям
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

#define GP_MIN_US 300000  // период, длиннее которого мотор можно резко тормозить или менять скорость

// создать планировщик с драйверами типа DRV и количеством осей AXLES
template <GS_driverType _DRV, uint8_t _AXLES>
class GPlanner {
   public:
    GPlanner() {
        setAcceleration(100);
        setMaxSpeed(300);
    }

    // ============================== MOTOR ==============================
    // добавить объект типа Stepper на ось axis, начиная с 0
    void addStepper(uint8_t axis, Stepper<_DRV>& stp) {
        if (axis < _AXLES) steppers[axis] = &stp;
    }

    // установить компенсацию люфта на ось axis в количестве шагов steps
    void setBacklash(uint8_t axis, uint16_t steps) {
        blash[axis] = steps;
    }

    // ============================== POWER ==============================
    // включить моторы
    void enable() {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->enable();
    }

    // выключить моторы
    void disable() {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->disable();
    }

    // переключить питание
    void power(bool v) {
        if (v) enable();
        else disable();
    }

    // ============================= PLANNER =============================
    // true - готов принять следующую точку маршрута
    bool ready() {
        if (readyF && !status) {
            readyF = false;
            return true;
        }
        return false;
    }

    // установка максимальной скорости планировщика в шаг/сек
    void setMaxSpeed(double speed) {
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
        // нет ускорения или крутим или медленно едем - дёргай ручник
        if (us == 0 || a == 0 || ((uint32_t)us << shift) > GP_MIN_US || status == 3 || !status) {
            brake();
            return;
        }
        if (status <= 2) {    // едем
            if (step > s2) {  // а мы уже тормозим!
                pause();      // значит флаг на паузу
                return;
            }
            us <<= shift;
            stopStep = 1000000ul / us;                         // наша скорость
            stopStep = (uint32_t)stopStep * stopStep / 2 / a;  // дистанция остановки. a не  может быть 0
            us10 = (uint32_t)us << 10;
            us >>= shift;
            status = 4;
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
    void home() {
        int32_t pos[_AXLES] = {};
        setTarget(pos);
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
    bool setTarget(const int32_t target[], GS_posType type = ABSOLUTE) {
        if (changeSett) {  // применяем настройки
            setMaxSpeed(nV);
            changeSett = 0;
        }

        S = 0;                                                  // путь
        for (uint8_t i = 0; i < _AXLES; i++) {                  // для всех осей
            tar[i] = target[i];                                 // запоминаем цель
            if (type == RELATIVE) tar[i] += steppers[i]->pos;   // если относительное смещение - прибавляем текущий pos
            dS[i] = abs(tar[i] - steppers[i]->pos);             // модуль ошибки по оси
            if (dS[i] > S) S = dS[i];                           // ищем максимальное отклонение
            int8_t dir = (steppers[i]->pos < tar[i]) ? 1 : -1;  // направление движения по оси
            if (blash[i] && steppers[i]->dir != dir) { // разворот! Учитываем люфт
                blash_buf[i] = blash[i];
            }
            steppers[i]->dir = dir;
        }
        if (S == 0) {       // путь == 0, мы никуда не едем
            readyF = true;  // готовы к следующей точке
            brake();        // стоп машина
            return 0;
        }

        shift = 0;
        for (; shift < 5; shift++) {
            if ((uint32_t)usMin >> shift < 200 || (0xfffffffl >> shift) < S) break;
        }
        shift--;
        int32_t subS = ((int32_t)S << shift) >> 1;
        for (int i = 0; i < _AXLES; i++) nd[i] = subS;  // записываем половину

        // расчёт точек смены характера движения
        // s1 - окончание разгона, s1-s2 - равномерное движение, s2 - торможение
        if (a > 0) {  // ускорение задано
            us <<= shift;
            if (us != 0 && us < GP_MIN_US) {  // мы движемся! ААА!
                int32_t v1 = 1000000L / us;
                if ((int32_t)V * V / a - ((int32_t)v1 * v1 / a >> 1) > S) {  // треугольник
                    s1 = ((int32_t)S >> 1) - ((int32_t)v1 * v1 / a >> 2);
                    s2 = s1;
                } else {  // трапеция
                    s1 = ((int32_t)V * V - (int32_t)v1 * v1) / (2L * a);
                    s2 = S - (int32_t)V * V / (2 * a);
                }
                so1 = (int32_t)v1 * v1 / (2 * a);
            } else {                           // не движемся
                if ((int32_t)V * V / a > S) {  // треугольник
                    s1 = S / 2L;
                    s2 = s1;
                } else {  // трапеция
                    s1 = (int32_t)V * V / (2 * a);
                    s2 = S - s1;
                }
                so1 = 0;
                us = us0;
            }
        } else {  // ускорение отключено
            s1 = 0;
            s2 = S;
            us = usMin;
        }
        // здесь us10 - us*1024 для повышения разрешения микросекунд в 1024 раз
        us10 = (uint32_t)us << 10;
        us >>= shift;
        step = substep = 0;
        readyF = false;
        status = 1;
        return 1;
    }

    bool setTarget(const int16_t target[], GS_posType type = ABSOLUTE) {
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
        if (speed == 0) {  // это куда ты собрался?
            brake();
            return;
        }
        speedAxis = axis;                          // запомнили ось
        steppers[axis]->dir = speed > 0 ? 1 : -1;  // направление
        us = 1000000.0 / abs(speed);               // период
        us >>= shift;
        status = 3;
    }

    // ============================= TICK =============================
    // тикер движения. Вернёт false если мотор остановлен. ~20..65us
    bool tick() {
        uint32_t now = micros();
        if (status > 0 && now - tmr >= us) {
            tmr = now;  //+= us;//= micros();
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

        // выбираем люфт
        bool skip = 0;
        for (uint8_t i = 0; i < _AXLES; i++) {
            if (blash_buf[i]) {
                blash_buf[i]--;
                steppers[i]->step();
                steppers[i]->pos += -steppers[i]->dir;
                if (!skip) skip = 1;
            }
        }
        if (skip) return 1;

        // здесь step - шаг вдоль общей линии траектории длиной S
        // шаги на проекциях получаются через алгоритм Брезенхема
        for (uint8_t i = 0; i < _AXLES; i++) {
            // http://members.chello.at/easyfilter/bresenham.html
            nd[i] -= dS[i];
            if (nd[i] < 0) {
                nd[i] += (int32_t)S << shift;
                steppers[i]->step();
            }
        }

        if (shift)
            if (++substep & ((1 << shift) - 1)) return status;  // пропускаем сабшаги

        step++;
        // плавная остановка
        if (status == 4) {
            stopStep--;
            us10 += 2ul * us10 / (4ul * stopStep + 1);  // торможение
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
            if (stopStep <= 0 || step >= S || us >= us0) brake();
            us >>= shift;
            return status;
        }

        // https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
        // здесь us10 - us*1024 для повышения разрешения микросекунд
        if (step < s1) {  // разгон
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
        } else if (step < s2) us = usMin;  // постоянная
        else if (step < S) {               // торможение
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
        } else {  // приехали
            if (status == 1) readyF = true;
            brake();
        }
        us >>= shift;
        return status;
    }

    uint32_t getPeriod() {
        return us << shift;
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
            uint32_t sa = (uint32_t)V * V / a / 2ul;           // расстояние разгона
            float dtf = sqrt(2.0 * sa / a) / GS_FAST_PROFILE;  // время участка профиля
            float s0 = a * dtf * dtf / 2.0;                    // первый участок профиля
            uint32_t dt = dtf * 1000000.0;                     // время участка в секундах
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
    // массивы шагов и периодов для быстрого профиля скорости
    uint32_t prfS[GS_FAST_PROFILE], prfP[GS_FAST_PROFILE];
#endif

    uint16_t blash[_AXLES] = {}, blash_buf[_AXLES] = {};

    uint32_t us;                                  // период шагов
    int32_t tar[_AXLES], nd[_AXLES], dS[_AXLES];  // цель, переменная Брезенхема, смещение по оси
    int32_t step, substep, S, s1, s2, so1;        // шаги на текущем участке, дробные шаги, длина участка, (s1,s2,so1) - для расчёта трапеций
    uint32_t tmr, us0, usMin, us10;               // таймер тика, время первого шага, мин. период шага, сдвинутый на 10 us
    uint16_t a, na;                               // ускорение, буфер ускорения для применения после остановки
    int16_t stopStep;                             // шагов до остановки
    float V, nV;                                  // скорость, буфер скорости для применения после остановки
    uint8_t status = 0, speedAxis = 0;            // статус, ось в режиме скорости
    uint8_t shift = 0;                            // сдвиг для повышения разрешения Брезенхема
    bool readyF = true;
    bool changeSett = 0;  // флаг, что изменились настройки
    Stepper<_DRV>* steppers[_AXLES];
};
#endif

/*
    Многоосевой планировщик траекторий для шаговых моторов
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
    
    AlexGyver, alex@alexgyver.ru
    https://alexgyver.ru/
    MIT License
    
    Версии:
    v1.0
*/

/*

GPlanner<драйвер, количество осей> planner;                 // объявяление
GPlanner<драйвер, количество осей, размер буфера> planner;  // + размер буфера (по умолч. 32)

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
*/

#ifndef _GyverPlanner2_h
#define _GyverPlanner2_h
#include <Arduino.h>
#include "StepperCore.h"
#include "FIFO.h"

#define GP_MIN_US 300000     // период, длиннее которого мотор можно резко тормозить или менять скорость

// создать планировщик с драйверами типа DRV и количеством осей AXLES
template < GS_driverType _DRV, uint8_t _AXLES, uint8_t _BUF = 32 >
class GPlanner2 {
public:
    GPlanner2() {
        setAcceleration(100);
        setMaxSpeed(300);
    }
    
    // ============================== MOTOR ==============================
    // добавить объект типа Stepper на ось axis, начиная с 0
    void addStepper(uint8_t axis, Stepper<_DRV> &stp) {
        if (axis < _AXLES) steppers[axis] = &stp;
    }

    // ============================= PLANNER =============================    
    // установка максимальной скорости планировщика в шаг/сек
    void setMaxSpeed(float nV) {
        V = nV;
        usMin = 1000000.0 / V;
    }
    
    // установка ускорения планировщика в шаг/сек^2
    void setAcceleration(uint16_t nA) {
        a = nA;
        if (a != 0) us0 = 0.676 * 1000000 * sqrt(2.0 / a);
        else us0 = 0;
    }
    
    // установить dt смены скорости, 0.0.. 1.0 по умолч. 0.3
    void setDtA(float newDta) {
        dtA = newDta;
    }

    // остановить плавно (с заданным ускорением)
    void stop() {
        if (us == 0 || status == 0 || status == 4) return;  // мы и так уже остановились, успокойся
        if (a == 0 || us > GP_MIN_US) {                     // нет ускорения или медленно едем - дёргай ручник
            brake();
            return;
        }
        status = 4;
        stopStep = 1000000ul / us;                              // наша скорость
        stopStep = (uint32_t)stopStep * stopStep / (2 * a);     // дистанция остановки
        us10 = (uint32_t)us << 10;
    }
    
    // начать работу
    void start() {
        if (status == 0) status = 1;  // если остановлен - проверяем буфер
    }

    // резко остановить моторы из любого режима
    void brake() {
        status = 0;
        us = 0;
        // пишем в буфер что мы остановились
        for (int i = 0; i < _AXLES; i++) bufP[i].set(0, steppers[i]->pos);
        bufV.set(0, 0);
    }
    
    // продолжить после остановки/паузы
    void resume() {
        if (status == 0) status = 1;  // если стоим ждём - проверить буфер
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
    
    // получить цель в шагах на оси axis
    int32_t getTarget(int axis) {
        return bufP[axis].get(1);
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
        status = 5;
    }
    
    // ============================= TICK =============================
    // тикер движения. Вернёт false если мотор остановлен. ~20..65us + ~10мс при пересчёте блока
    bool tick() {
        checkBuffer();
        uint32_t now = micros();
        if (status > 1 && now - tmr >= us) {
            tmr = now;//+= us;//= micros();
            tickManual();
        }
        return status > 1;
    }
    
    bool tickManual() {
        // режим постоянной скорости
        if (status == 5) {
            steppers[speedAxis]->step();
            return 1;
        }
        // здесь step - шаг вдоль общей линии траектории длиной S
        // шаги на проекциях получаются через алгоритм Брезенхема
        step++;
        for (uint8_t i = 0; i < _AXLES; i++) {
            // http://members.chello.at/easyfilter/bresenham.html
            nd[i] -= dS[i];
            if (nd[i] < 0) {
                nd[i] += S;
                steppers[i]->step();
            }
        }
        
        // плавная остановка
        if (status == 4) {
            stopStep--;
            us10 += 2ul * us10 / (4ul * stopStep + 1);    // торможение
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
            if (step >= S) {
                next();
                setTarget();
                us = us10 >> 10;
            }
            if (stopStep <= 0 || us >= us0) brake();            
            return status > 1;
        }

        // https://www.embedded.com/generate-stepper-motor-speed-profiles-in-real-time/
        // здесь us10 - us*1024 для повышения разрешения микросекунд
        if (step < s1) {                                    // разгон
            us10 -= 2ul * us10 / (4ul * (step + so1) + 1);
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
        }
        else if (step < s2) us = usMin;                     // постоянная
        else if (step < S) {                                // торможение
            us10 += 2ul * us10 / (4ul * (S - step) + 1);
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
        } else {                                            // приехали
            if (status == 3) {              // достигли конечной точки
                readyF = true;
                status = 0;
            } else status = 1;              // иначе проверяем буфер
            next();
        }
        return status > 1;
    }
    
    // обработчик буфера. Сам вызывается в tick. Нужно вызывать вручную при работе с tickManual
    bool checkBuffer() {
        if (status == 1) {
            if (bufV.available() > 1) {
                if (bufV.get(0) == 0) calculateBlock();
                if (setTarget()) return 1;                
            }            
        }
        return 0;
    }
    
    // флаг достижения точки остановки. После неё нужно вызывать resume
    bool ready() {
        if (readyF) {
            readyF = false;
            return true;
        } return false;
    }
    
    // true - в буфере планировщика есть место под новю точку
    bool available() {
        return bufV.availableForWrite();
    }
    
    // добавить новую точку. Массив координат, флаг окончания и абсолютный/относительный
    void addTarget(int32_t tar[], uint8_t l, GS_posType type = ABSOLUTE) {
        if (type = ABSOLUTE) for (int i = 0; i < _AXLES; i++) bufP[i].add(tar[i]);        
        else for (int i = 0; i < _AXLES; i++) bufP[i].add(tar[i] + bufP[i].get(-1));
        bufL.add(l);
        bufV.add(0);
        bufS.add(0);
    }
    
    void addTarget(int16_t tar[], uint8_t l, GS_posType type = ABSOLUTE) {
        int32_t ntar[_AXLES];
        for (int i = 0; i < _AXLES; i++) ntar[i] = (int32_t)tar[i];
        addTarget(ntar, l, type);
    }
    
    // пересчитать буфер. ~10ms на размер 32 и 2 оси
    void calculateBlock() {
        // поиск максимальной конечной скорости
        for (int i = 0; i < bufV.available() - 1; i++) {
            uint32_t sqSum = 0;
            int32_t dn[_AXLES];            
            for (int j = 0; j < _AXLES; j++) {
                dn[j] = bufP[j].get(i + 1) - bufP[j].get(i);
                sqSum += (int32_t)dn[j] * dn[j];
            }
            uint32_t s1 = sqrt(sqSum);
            bufS.set(i, s1);
            if (bufL.get(i + 1) == 1) break;
            if (a == 0) {
                bufV.set(i + 1, uint16_t(V));
                continue;
            }
            if (s1 == 0) continue;

            if (i < bufV.available() - 2) {
                int32_t multSum = 0;
                int32_t dn1[_AXLES];
                sqSum = 0;
                for (int j = 0; j < _AXLES; j++) {
                    dn1[j] = bufP[j].get(i + 2) - bufP[j].get(i + 1);
                    sqSum += (int32_t)dn1[j] * dn1[j];
                    multSum += (int32_t)dn[j] * dn1[j];
                }
                uint32_t s2 = sqrt(sqSum);
                if (s2 == 0) continue;
                float cosa = -(float)multSum / s1 / s2;
                float v2;
                if (cosa < -0.95) v2 = V;
                else v2 = (float)a * dtA / sqrt(2.0 * (1 + cosa));
                v2 = min(v2, V);
                bufV.set(i + 1, v2);
            }
        }

        // рекурсивно уменьшаем конечные скорости на участке
        for (int i = 0; i < bufV.available() - 1; i++) {
            uint16_t v0 = bufV.get(i);
            uint16_t v1 = bufV.get(i + 1);
            uint16_t maxV = sqrt(2.0 * a * bufS.get(i) + v0 * v0);
            if (v1 > v0 && maxV < v1) {
                bufV.set(i + 1, maxV);
            } else if (v1 < v0 && maxV > v1) {
                int16_t count = 0;
                while (1) {
                    uint32_t minV = sqrt(2ul * a * bufS.get(i + count) + (uint32_t)bufV.get(i + count + 1) * bufV.get(i + count + 1));        
                    if (minV >= bufV.get(i + count)) break;
                    else bufV.set(i + count, minV);          
                    count--;
                }
            }
        }
    }
    
    // время до следующего tick, мкс
    uint32_t getPeriod() {
        return us;
    }
    
    // статус планировщика
    uint8_t getStatus() {
        return status;
    }
    
    // ============================== PRIVATE ==============================
private:
    void next() {
        for (int i = 0; i < _AXLES; i++) bufP[i].next();
        bufL.next();
        bufV.next();
        bufS.next();
    }
    
    // установить цель в шагах и начать движение. ~100 us
    bool setTarget() {   
        tmr = micros();
        for (uint8_t i = 0; i < _AXLES; i++) {                      // для всех осей            
            dS[i] = abs(bufP[i].get(1) - steppers[i]->pos);         // модуль ошибки по оси      
            steppers[i]->dir = steppers[i]->pos < bufP[i].get(1) ? 1 : -1;  // направление движения по оси
        }
        S = bufS.get(0);
        if (S == 0) {       // путь == 0, мы никуда не едем
            status = 1;     // на буфер
            next();
            return 0;
        }
        
        for (int i = 0; i < _AXLES; i++) nd[i] = S / 2u;    // записываем половину
        
        if (a > 0) {
            int16_t v1 = bufV.get(0);      // скорость начала отрезка
            int16_t v2 = bufV.get(1);    

            if (2L * V * V - v1 * v1 - v2 * v2 > 2L * a * S) {  // треугольник
                s1 = (2L * a * S + (int32_t)v2 * v2 - (int32_t)v1 * v1) / (4L * a);
                s2 = 0;
            } else {          // трапеция
                s1 = ((int32_t)V * V - (int32_t)v1 * v1) / (2L * a);
                s2 = S - ((int32_t)V * V - (int32_t)v2 * v2) / (2L * a);
            }
            so1 = (int32_t)v1 * v1 / (2 * a);
            so2 = (int32_t)v2 * v2 / (2 * a);
            if (status != 4) {
                if (v1 == 0) us = us0;
                else us = 1000000ul / v1;
            }
        } else {
            so1 = so2 = 0;
            s1 = 0;
            s2 = S;
            us = usMin;
        }

        step = 0;
        readyF = false;
        if (status != 4) {    // если это не стоп
            if (bufL.get(1) == 1) status = 3;
            else status = 2;
            us10 = us << 10;
        }  
        return 1;
    }

    uint32_t us;
    int32_t nd[_AXLES], dS[_AXLES];
    int32_t step, S, s1, s2, so1, so2;
    uint32_t tmr, us0, usMin, us10;
    uint16_t a;
    int16_t stopStep;
    float V;    
    uint8_t status = 0, speedAxis = 0, maxAx;
    bool readyF = true;
    float dtA = 0.3;
    
    Stepper<_DRV>* steppers[_AXLES];    
    FIFO<int32_t, _BUF> bufP[_AXLES];
    FIFO<uint8_t, _BUF> bufL;
    FIFO<uint16_t, _BUF> bufV;
    FIFO<uint32_t, _BUF> bufS;
};

#endif
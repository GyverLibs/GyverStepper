/*
    Многоосевой планировщик траекторий для шаговых моторов
    - ПЛАНИРОВАНИЕ СКОРОСТИ НА МАРШРУТЕ. НАСТРАИВАЕМЫЙ БУФЕР
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
    
    void clearBuffer();     // очистить буфер. Вызывать, когда планировщик остановлен!
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
        if (a > 0) {
            if (step < s1) {                                    // разгон
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
            else if (step < s2) us = usMin;                     // постоянная
            else if (step < S) {                                // торможение
                #ifndef GS_FAST_PROFILE
                us10 += 2ul * us10 / (4ul * (S - step + so2) + 1);
                us = (uint32_t)us10 >> 10;
                us = constrain(us, usMin, us0);
                #else
                if ((S - step + so2) >= prfS[GS_FAST_PROFILE - 1]) us = usMin;
                else {
                    int j = 0;
                    while ((S - step + so2) >= prfS[j]) j++;
                    us = prfP[j];
                }
                #endif
            }
        }
        if (step >= S) {                    // приехали
            if (status == 3) {              // достигли конечной точки
                readyF = true;
                status = 0;
            } else status = 1;              // иначе проверяем буфер
            next();
        }
        return (status > 1);
    }
    
    // обработчик буфера. Сам вызывается в tick. Нужно вызывать вручную при работе с tickManual
    bool checkBuffer() {
        if (status == 1) {      // достигли цели, запрашиваем новую
            if (bufL.available() > 1) {
                if (--block < 0) block = 0;
                // в движении с ускорением пересчитываем путь, если остановились или есть на это время
                if (a > 0 && (bufV.get(0) == 0 || (block == 0 && !changeSett && us > blockCalc))) calculateBlock();
                if (setTarget()) return 1;
            }            
        }
        return 0;
    }
    
    // флаг достижения точки остановки. После неё нужно вызывать resume
    bool ready() {
        if (readyF && !status) {
            readyF = false;
            return 1;
        } return 0;
    }
    
    // true - в буфере планировщика есть место под новю точку
    bool available() {
        return bufV.availableForWrite();
    }
    
    // добавить новую точку. Массив координат, флаг окончания и абсолютный/относительный
    void addTarget(int32_t tar[], uint8_t l, GS_posType type = ABSOLUTE) {
        if (type == ABSOLUTE) for (int i = 0; i < _AXLES; i++) bufP[i].add(tar[i]);        
        else for (int i = 0; i < _AXLES; i++) bufP[i].add(tar[i] + bufP[i].getLast());
        bufL.add(l);
        bufV.add(0);
        bufS.add(0);
    }
    
    void addTarget(int16_t tar[], uint8_t l, GS_posType type = ABSOLUTE) {
        int32_t ntar[_AXLES];
        for (int i = 0; i < _AXLES; i++) ntar[i] = (int32_t)tar[i];
        addTarget(ntar, l, type);
    }
        
    // время до следующего tick, мкс
    uint32_t getPeriod() {
        return us;
    }
    
    // статус планировщика
    uint8_t getStatus() {
        return status;
    }
    
    // очистить буфер
    void clearBuffer() {
        for (int i = 0; i < _AXLES; i++) bufP[i].clear();
        bufL.clear();
        bufV.clear();
        bufS.clear();
    }
    
    // ============================== PRIVATE ==============================
private:
    // пересчитать скорости
    void calculateBlock() {
        if (changeSett && bufV.get(0) == 0) {   // параметры движения изменились, а мы стоим
            noInterrupts();
            uint8_t stBuf = status;
            status = 0;
            setMaxSpeed(nV);
            status = stBuf;
            interrupts();
            changeSett = 0;
        }
        
        uint32_t calcTime = micros();
        block = _BUF / 2;   // через половину буфера попробуем сделать перерасчёт пути
        uint32_t nextS = calcS(0);
        
        // поиск максимальной конечной скорости
        for (uint16_t i = 0; i < bufV.available() - 1; i++) {
            int32_t dn0[_AXLES];
            for (int j = 0; j < _AXLES; j++) dn0[j] = bufP[j].get(i + 1) - bufP[j].get(i);  // расстояние между точками (катеты)
            uint32_t S1 = nextS;                                                            // "гипотенуза" (на 1 шаге посчитана выше)
            bufS.set(i, S1);                                                                // записали в буфер
            if (bufL.get(i + 1) == 1) break;                                                // последняя точка - выходим, потом посчитаем
            if (S1 == 0) continue;                                                          // гипотенуза 0 - пропускаем

            if (i < bufV.available() - 2) {                         // для следующих точек (+1)
                int32_t multSum = 0;
                // складываем перемноженные катеты для расчёта косинуса
                for (int j = 0; j < _AXLES; j++) multSum += (int32_t)dn0[j] * (bufP[j].get(i + 2) - bufP[j].get(i + 1));
                nextS = calcS(i + 1);                               // гипотенуза
                if (nextS == 0) continue;                           // гипотенуза 0 - пропускаем
                float cosa = -(float)multSum / S1 / nextS;          // косинус угла между отрезками пути
                float Vm;                                           // макс. скорость в повороте
                if (cosa < -0.95) Vm = V;                           // пролетаем по прямой
                else {
                    Vm = (float)a * dtA / sqrt(2.0 * (1 + cosa));   // считаем
                    Vm = min(Vm, V);                                // на всякий случай ограничим
                }
                bufV.set(i + 1, Vm);                                // пишем в буфер
            }
        }

        // уменьшаем переходные скорости на траектории
        for (uint16_t i = 0; i < bufV.available() - 1; i++) {
            uint32_t v0 = bufV.get(i);
            uint32_t v1 = bufV.get(i + 1);
            uint32_t maxV = sqrt(2L * a * bufS.get(i) + (uint32_t)v0 * v0);
            if (v1 > v0 && maxV < v1) bufV.set(i + 1, maxV);    // всё в порядке
            else if (v1 < v0 && maxV > v1) {                    // идём назад по буферу и уменьшаем скорости
                int16_t count = 0;
                while (true) {
                    uint32_t minV = bufV.get(i + count + 1);
                    minV *= (uint32_t)minV;
                    minV = sqrt(2ul * a * bufS.get(i + count) + minV);        
                    if (minV >= bufV.get(i + count)) break;
                    else bufV.set(i + count, minV);                 
                    count--;
                }
            }
        }
        // находим среднее время пересчёта, чтобы вызвать его между точками (когда появится возможность)
        blockCalc = (blockCalc + micros() - calcTime) / 2;
    }
    
    void next() {
        for (int i = 0; i < _AXLES; i++) bufP[i].next();
        bufL.next();
        bufV.next(FIFO_WIPE);   // обнуляем использованную ячейку
        bufS.next();
    }
    
    // установить цель в шагах и начать движение. ~100 us
    bool setTarget() {   
        tmr = micros();
        for (uint8_t i = 0; i < _AXLES; i++) {                      // для всех осей            
            dS[i] = abs(bufP[i].get(1) - steppers[i]->pos);         // модуль ошибки по оси      
            steppers[i]->dir = steppers[i]->pos < bufP[i].get(1) ? 1 : -1;  // направление движения по оси
        }
        
        if (a == 0) S = calcS(0);
        else S = bufS.get(0);
        
        if (S == 0) {       // путь == 0, мы никуда не едем
            status = 1;     // на буфер
            next();
            return 0;
        }
        
        for (int i = 0; i < _AXLES; i++) nd[i] = S / 2u;    // записываем половину
        
        if (a > 0) {
            int32_t v1 = bufV.get(0);   // скорость начала отрезка
            int32_t v2 = bufV.get(1);   // скорость конца отрезка

            if (2L * V * V - (int32_t)v1 * v1 - (int32_t)v2 * v2 > 2L * a * S) {  // треугольник
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
            s1 = 0;
            s2 = S;
            so1 = so2 = 0;
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
    
    uint32_t calcS(int i) {
        uint32_t sqSum = 0;
        int32_t dj;
        for (int j = 0; j < _AXLES; j++) {
            dj = bufP[j].get(i + 1) - bufP[j].get(i);
            sqSum += (int32_t)dj * dj;
        }
        return sqrt(sqSum);
    }
    
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
    int32_t nd[_AXLES], dS[_AXLES];
    int32_t step, S, s1, s2, so1, so2;
    uint32_t tmr, us0, usMin, us10;
    uint16_t a, na;
    int16_t stopStep;
    float V, nV;    
    uint8_t status = 0, speedAxis = 0, maxAx;
    bool readyF = true;
    float dtA = 0.3;
    
    int8_t block = 100;
    uint32_t blockCalc = 20000;
    bool changeSett = 0;
    
    
    Stepper<_DRV>* steppers[_AXLES];    
    FIFO<int32_t, _BUF> bufP[_AXLES];
    FIFO<uint8_t, _BUF> bufL;
    FIFO<uint16_t, _BUF> bufV;
    FIFO<uint32_t, _BUF> bufS;
};

#endif
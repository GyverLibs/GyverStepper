/*
    Ядро библиотеки для управления шаговыми моторами:
    - 4 фазные и STEP DIR драйверы
    - Поддержка пина EN
    - Виртуальный драйвер
    - Быстрый алгоритм IO для AVR
    
    AlexGyver, alex@alexgyver.ru
    https://alexgyver.ru/
    MIT License
*/

/*
    // ======== ИНИЦИАЛИЗАЦИЯ ========
    Stepper<STEPPER2WIRE> stepper(step, dir);						// драйвер step-dir
    Stepper<STEPPER2WIRE> stepper(step, dir, en);					// драйвер step-dir + пин enable
    Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4);			// драйвер 4 пин
    Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4, en);		// драйвер 4 пин + enable
    Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4);		// драйвер 4 пин полушаг
    Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4, en);	// драйвер 4 пин полушаг + enable

    Stepper<STEPPER2WIRE, STEPPER_VIRTUAL> stepper;					// виртуальный драйвер step-dir
    Stepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper;					// виртуальный драйвер 4 пин

    // ============ КЛАСС ============
    void step();                                // сделать шаг
    void invertEn(bool val);                    // инвертировать поведение EN пина
    void reverse(bool val);                     // инвертировать направление мотора
    void disable();                             // отключить питание и EN
    void enable();                              // включить питание и EN
    void attachStep(void (*handler)(uint8_t));  // подключить обработчик шага
    void attachPower(void (*handler)(bool));    // подключить обработчик питания

    int32_t pos;                                // текущая позиция в шагах
    int8_t dir;                                 // направление (1, -1)
*/

#ifndef _StepperCore_h
#define _StepperCore_h
#include <Arduino.h>
#include "GStypes.h"

#ifndef DRIVER_STEP_TIME
#define DRIVER_STEP_TIME 4
#endif

#define _PINS_AMOUNT ( (_TYPE == STEPPER_PINS) ? (_DRV == 0 ? 2 : 4) : (0) )

template <GS_driverType _DRV, GS_driverType _TYPE = STEPPER_PINS>
class Stepper {
public:
    Stepper(uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255, uint8_t pin4 = 255, uint8_t pin5 = 255) {
        if (_TYPE == STEPPER_PINS) {
            if (_DRV == STEPPER2WIRE) {
                configurePin(0, pin1);
                configurePin(1, pin2);
                if (pin3 != 255) {
                    _enPin = pin3;
                    pinMode(_enPin, OUTPUT);
                }
            } else {
                configurePin(0, pin1);
                configurePin(1, pin2);
                configurePin(2, pin3);
                configurePin(3, pin4);
                if (pin5 != 255) {
                    _enPin = pin5;
                    pinMode(_enPin, OUTPUT);
                }
            }
        }
    }

    // сделать шаг
    void step() {
        pos += dir;
        if (_DRV == STEPPER2WIRE) {     // ~4 + DRIVER_STEP_TIME us
            stepDir();
        } else {                        // ~5.5 us
            thisStep += (_globDir ? dir : -dir);
            step4();
        }
    }
    
    // инвертировать поведение EN пина
    void invertEn(bool val) {
        _enDir = val;
    }
    
    // инвертировать направление мотора
    void reverse(bool val) {
        _globDir = val;
    }
    
    // отключить питание и EN
    void disable() {
        if (_TYPE == STEPPER_PINS) {
            if (_DRV == STEPPER4WIRE || _DRV == STEPPER4WIRE_HALF) {
                setPin(0, 0);
                setPin(1, 0);
                setPin(2, 0);
                setPin(3, 0);
            }
            if (_enPin != 255) digitalWrite(_enPin, !_enDir);
        } else if (*_power) _power(0);
    }
    
    // включить питание и EN
    void enable() {
        if (_TYPE == STEPPER_PINS) {
            // подадим прошлый сигнал на мотор, чтобы вал зафиксировался
            if (_DRV == STEPPER4WIRE || _DRV == STEPPER4WIRE_HALF) step4();	
            if (_enPin != 255) digitalWrite(_enPin, _enDir);
        } else if (*_power) _power(1);
    }
    
    // подключить обработчик шага
    void attachStep(void (*handler)(uint8_t)) {
        _step = handler;
    }
    
    // подключить обработчик питания
    void attachPower(void (*handler)(bool)) {
        _power = handler;
    }
    
    int32_t pos = 0;
    int8_t dir = 1;

private:        
    // настройка пина
    void configurePin(int num, uint8_t pin) {
        pinMode(pin, OUTPUT);
        #ifdef __AVR__
        _port_reg[num] = portOutputRegister(digitalPinToPort(pin));
        _bit_mask[num] = digitalPinToBitMask(pin);
        #else
        _pins[num] = pin;
        #endif
    }

    // быстрая установка пина
    void setPin(int num, bool state) {
        #ifdef __AVR__
        if (state) *_port_reg[num] |= _bit_mask[num];
        else *_port_reg[num] &= ~_bit_mask[num];
        #else
        digitalWrite(_pins[num], state);
        #endif
    }
    
    // шаг для 4 фаз
    void step4() {        
        if (_TYPE == STEPPER_PINS) {
            if (_DRV == STEPPER4WIRE) {
                // 0b11 берёт два бита, т.е. формирует 0 1 2 3 0 1..
                switch (thisStep & 0b11) {			
                case 0: setPin(0, 1); setPin(1, 0); setPin(2, 1); setPin(3, 0); break;	// 1010
                case 1: setPin(0, 0); setPin(1, 1); setPin(2, 1); setPin(3, 0); break;	// 0110
                case 2: setPin(0, 0); setPin(1, 1); setPin(2, 0); setPin(3, 1); break;	// 0101
                case 3: setPin(0, 1); setPin(1, 0); setPin(2, 0); setPin(3, 1); break;	// 1001
                }			
            } else if (_DRV == STEPPER4WIRE_HALF) {
                // 0b111 берёт три бита, т.е. формирует 0 1 2 4 5 6 7 0 1 2..
                switch (thisStep & 0b111) {
                case 0: setPin(0, 1); setPin(1, 0); setPin(2, 0); setPin(3, 0); break;	// 1000
                case 1: setPin(0, 1); setPin(1, 0); setPin(2, 1); setPin(3, 0); break;	// 1010
                case 2: setPin(0, 0); setPin(1, 0); setPin(2, 1); setPin(3, 0); break;	// 0010
                case 3: setPin(0, 0); setPin(1, 1); setPin(2, 1); setPin(3, 0); break;	// 0110
                case 4: setPin(0, 0); setPin(1, 1); setPin(2, 0); setPin(3, 0); break;	// 0100
                case 5: setPin(0, 0); setPin(1, 1); setPin(2, 0); setPin(3, 1); break;	// 0101
                case 6: setPin(0, 0); setPin(1, 0); setPin(2, 0); setPin(3, 1); break;	// 0001
                case 7: setPin(0, 1); setPin(1, 0); setPin(2, 0); setPin(3, 1); break;	// 1001
                }
            }
        } else if (*_step) {
            if (_DRV == STEPPER4WIRE) {	
                switch (thisStep & 0b11) {			
                case 0: _step(0b1010); break;	// 1010
                case 1: _step(0b0110); break;	// 0110
                case 2: _step(0b0101); break;	// 0101
                case 3: _step(0b1001); break;	// 1001
                }			
            } else if (_DRV == STEPPER4WIRE_HALF) {
                switch (thisStep & 0b111) {
                case 0: _step(0b1000); break;	// 1000
                case 1: _step(0b1010); break;	// 1010
                case 2: _step(0b0010); break;	// 0010
                case 3: _step(0b0110); break;	// 0110
                case 4: _step(0b0100); break;	// 0100
                case 5: _step(0b0101); break;	// 0101
                case 6: _step(0b0001); break;	// 0001
                case 7: _step(0b1001); break;	// 1001
                }
            }
        }
    }
    
    // шажочек степдир
    void stepDir() {
        if (_TYPE == STEPPER_PINS) {
            setPin(1, (dir > 0 ? _globDir : !_globDir) );	// DIR
            setPin(0, 1);	// step HIGH
            if (DRIVER_STEP_TIME > 0) delayMicroseconds(DRIVER_STEP_TIME);
            setPin(0, 0);	// step LOW
        } else if (*_step) {
            _step(dir > 0 ? _globDir : !_globDir);
        }
    }
    
    int8_t _enPin = 255;
    bool _enDir = false;
    bool _globDir = false;
    int8_t thisStep = 0;
    
    void (*_step)(uint8_t a) = NULL;
    void (*_power)(bool a) = NULL;
    
#ifdef __AVR__
    volatile uint8_t *_port_reg[_PINS_AMOUNT];
    volatile uint8_t _bit_mask[_PINS_AMOUNT];
#else
    uint8_t _pins[_PINS_AMOUNT];
#endif
};

#endif

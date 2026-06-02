This is an automatic translation and may be incorrect in some places. See the source README and examples for authoritative information.

[![latest](https://img.shields.io/github/v/release/GyverLibs/GyverStepper.svg?color=brightgreen)](https://github.com/GyverLibs/GyverStepper/releases/latest/download/GyverStepper.zip)
[![PIO](https://badges.registry.platformio.org/packages/gyverlibs/library/GyverStepper.svg)](https://registry.platformio.org/libraries/gyverlibs/GyverStepper)
[![Foo](https://img.shields.io/badge/Website-AlexGyver.ru-blue.svg?style=flat-square)](https://alexgyver.ru/)
[![Foo](https://img.shields.io/badge/%E2%82%BD%24%E2%82%AC%20%D0%9F%D0%BE%D0%B4%D0%B4%D0%B5%D1%80%D0%B6%D0%B0%D1%82%D1%8C-%D0%B0%D0%B2%D1%82%D0%BE%D1%80%D0%B0-orange.svg?style=flat-square)](https://alexgyver.ru/support_alex/)
[![Foo](https://img.shields.io/badge/README-ENGLISH-blueviolet.svg?style=flat-square)](https://github-com.translate.goog/GyverLibs/GyverStepper?_x_tr_sl=ru&_x_tr_tl=en)  

[![Foo](https://img.shields.io/badge/ПОДПИСАТЬСЯ-НА%20ОБНОВЛЕНИЯ-brightgreen.svg?style=social&logo=telegram&color=blue)](https://t.me/GyverLibs)

# GyverStepper
Production library for controlling stepper motors with Arduino
- Support for 4-pin (step and half) and STEP-DIR drivers
- Automatic power outage when achieving the goal
- Working hours:
    - Rotating at a given speed. Smooth acceleration and acceleration braking
    - Follow the acceleration and speed limit position
    - Follow the position at a given speed (without acceleration)
- Fast step control algorithm
- Support for “virtual” drivers
- Built-in multi-axis trajectory planner

### Compatibility
Compatible with all Arduino platforms (Arduino features are used)

### Documentation.
There's a library[extended documentation](https://alexgyver.ru/GyverStepper/)

## Contents
- [Installation](#install)
- [StepperCore](#core)
- [GyverStepper](#stepper)
- [GyverStepper2](#stepper2)
- [GyverPlanner](#planner)
- [GyverPlanner2](#planner2)
- [Versions](#versions)
- [Bugs and feedback](#feedback)

## Why are there so many things?!
The library contains a set of tools for different scenarios of working with stepper motors
- *StepperCore.h* [class **Stepper**]: the kernel of all other classes, can quickly click pins (AVR) and take one step for a configured driver type. Supports 4 step/half phases as well as step-dir drivers.
- *GyverStepper.h* [class **GStepper**]: basic heavy library, lots of settings. The movement of one motor with acceleration to a given position or rotation at a given speed. Not very optimal work in interrupting the timer.
- *GyverStepper2.h* [class **GStepper2**]: a new lightweight version of GyverStepper, almost completely compatible with it. A more optimal integer hybrid acceleration motion algorithm, lightweight. Optimized to work in timer interruption.
- *GyverPlanner.h* [class **GPlanner**]: multi-axis trajectory planner, acceleration motion (2 orders of magnitude). Stop at every point. Optimal work in interrupting the timer.
- *GyverPlanner2.h* [class **GPlanner2**]: multi-axis trajectory planner, acceleration motion (2 order). Planning speed on the route, optimal movement on points. Optimal work in interrupting the timer.

<a id="install"></a>
## Installation
- The library can be found under the name **GyverStepper** and installed through the library manager in:
    - Arduino IDE
    - Arduino IDE v2
    - PlatformIO
- [Download the library](https://github.com/GyverLibs/GyverStepper/archive/refs/heads/main.zip).zip archive for manual installation:
    - Unpack and put in *C:\Program Files (x86)\Arduino\libraries* (Windows x64)
    - Unpack and put in *C:\Program Files\Arduino\libraries* (Windows x32)
    - Unpack and put in *Documents/Arduino/libraries/ *
    - (Arduino IDE) Automatic installation from .zip: *Sketch/Connect library/Add .ZIP library...* and specify downloaded archive
- Read more detailed instructions for installing libraries[here](https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)
### Update
- I recommend always updating the library: new versions fix errors and bugs, as well as optimize and add new features.
- Through the library manager IDE: find the library as when installing and click "Update"
- Manually: **Delete the folder with the old version** and then put the new one in its place. “Replacement” can not be done: sometimes new versions delete files that will remain when replaced and can lead to errors!

<a id="core"></a>
## StepperCore
### Description
The core of the library for controlling stepping motors:
- 4 phase and STEP DIR drivers
- Support for EN pin
- Virtual driver
- Fast IO Algorithm for AVR

<details >>
<summary>Return</summary >>

### Initialization (StepperCore)
```cpp
Stepper<STEPPER2WIRE> stepper(step, dir);						// step-dir
Stepper<STEPPER2WIRE> stepper(step, dir, en);					// Driver step-dir + pin enable
Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4);			// driver
Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4, en);		// driver 4 pin + enable
Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4);		// driver 4 pin halfway
Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4, en);	// driver 4 pin halfway + enable

Stepper<STEPPER2WIRE, STEPPER_VIRTUAL> stepper;					// virtual step-dir driver
Stepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper;					// 4-pin virtual driver
```

### Use (StepperCore)
```cpp
void step();                                // step up
void invertEn(bool val);                    // invert
void reverse(bool val);                     // reverse
void disable();                             // switch off power and EN
void enable();                              // power up and EN
void power(bool);                           // switch off
void attachStep(void (*handler)(uint8_t));  // plug in
void attachPower(void (*handler)(bool));    // plug in

int32_t pos;                                // stepwise
int8_t dir;                                 // direction (1, -1)
```

### Example
For more examples see **examples**!
```cpp
#include <StepperCore.h>
Stepper<STEPPER2WIRE> stepper(2, 3);

void setup() {
  stepper.dir = 1;  // or -1
  stepper.pos = 0;  // positioning
}

void loop() {
  // manually
  stepper.step();   // step up
  delay(10);
}
```
</details>

<a id="stepper"></a>
## GyverStepper and GyverStepper2
- GStepper2 is much lighter and more optimized
- GStepper2 is designed to move along the points of the trajectory, but the “tracking” mode also works.
- GStepper2 optimized for timer interrupt
- GStepper2 uses a smooth motor motion algorithm by default
- GStepper2 is likely to be slightly more loaded on the microcontroller during operation than GStepper. But much less than GStepper in SMOOTH ALGORITHM mode.

### How does GStepper work?
- Fast algorithm (default): The library has two timers: a step timer (different period) and a scheduler timer (10-30 milliseconds). The scheduler timer recalculates the trajectory and calculates the direction of movement and the current required speed. This speed results in a new period for the step timer.
- Smooth algorithm (SMOOTH ALGORITHM setting): one step timer works, at each step the speed and time are recalculated to the next step.

### How does GStepper2 work?
- Smooth profile (default): It uses the smooth motion algorithm SMOOTH ALGORITHM as in GStepper, but optimized and accelerated by 2-3 times. All heavy trajectory calculations are performed when a new target position is specified in setTarget. Further on the step timer, the motor moves and quickly calculates the new time of the next step. This allows you to spend a minimum of time interrupting the timer if it is used.
- Quick profile (GS FAST PROFILE setting): the same thing, but the step timer does not calculate, but selects the period from a pre-calculated table, which allows you to move with acceleration up to speeds of 30,000 steps per second.

## GyverStepper
### Description
Library for controlling stepping motors with Arduino
- Support for 4-pin (step and half) and STEP-DIR drivers
- Automatic power outage when achieving the goal
- Working hours:
    - Rotating at a given speed. Smooth acceleration and acceleration braking
    - Follow the acceleration and speed limit position
    - Follow the position at a given speed (without acceleration)

<details >>
<summary>Return</summary >>

### Initialization
```cpp
// steps - steps per turn of the shaft (for calculations with degrees)
// step, dir, pin1, pin2, pin3, pin4 - any GPIO
// en - PIN disabling the driver, any GPIO
GStepper<STEPPER2WIRE> stepper(steps, step, dir);                   // step-dir
GStepper<STEPPER2WIRE> stepper(steps, step, dir, en);               // Driver step-dir + pin enable
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4);      // driver
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4, en);  // driver 4 pin + enable
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4);     // driver 4 pin halfway
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4, en); // driver 4 pin halfway + enable
GStepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper(2048);                  // Virtual driver, specify only the number of steps
```

### Use of use
```cpp
// Note: "default" means "even if no function is called"

// There is a movement of the motor, call as often as possible!
// It has a built-in timer.
// Returns true if the motor is moving towards the target or spinning KEEP SPEED
bool tick();

// Returns the same thing as the tick, i.e. whether the motor turns or not.
bool getState();

// Invert the direction of the motor - true (in silent. false)
void reverse(bool dir);

// Invert the behavior of EN PIN - true (by omission. false)
void invertEn(bool rev);

// Mode setting, mode:
// FOLLOW POS - following the position setTarget(...)
// KEEP SPEED - SetSpeed (...)
void setRunMode(GS_runMode mode);

// Setting the current position of the motor in steps and degrees
void setCurrent(long pos);
void setCurrentDeg(float pos);

// Reading the current motor position in steps and degrees
long getCurrent();
float getCurrentDeg();

// setting the target position in steps and degrees (for FOLLOW POS)
// Type - ABSOLUTE or RELATIVE, the default is ABSOLUTE
void setTarget(long pos);
void setTarget(long pos, GS_posType type);
void setTargetDeg(float pos);
void setTargetDeg(float pos, GS_posType type);

// Getting the target position in steps and degrees
long getTarget();
float getTargetDeg();

// Set the maximum speed (module) in steps/second and degrees/second (for FOLLOW POS mode)
// Shut up. 300.
void setMaxSpeed(float speed);
void setMaxSpeedDeg(float speed);

// Set acceleration in steps and degrees per second (for FOLLOW POS mode).
// At 0, the acceleration is switched off and the motor is running.
// Continuous maximum speed profile setMaxSpeed().
// Shut up. 300.
void setAcceleration(int accel);
void setAccelerationDeg(float accel);

// Auto-disconnect EN when reaching position - true (by default. false)
void autoPower(bool mode);

// Smooth stop with specified acceleration
void stop();

// Hard stop.
void brake();

// Hard stop + reset position to 0 (for tips)
void reset();

// Setting the target speed in steps / second and degrees / second (for KEEP SPEED mode)
void setSpeed(float speed);
void setSpeedDeg(float speed);

// Getting the target speed in steps / second and degrees / second (for KEEP SPEED mode)
float getSpeed();
float getSpeedDeg();

// Power up the engine (pin EN)
void enable();

// Turn off the engine (pin EN)
void disable();

// Returns the minimum tick period of the motor in microseconds at setMaxSpeed() speed.
// Can be used to configure timer interrupts with a tick() in the handler (see example timerISR)
uint16_t getMinPeriod();

// The current "tick" period for debugging and all that.
uint16_t stepTime;

// connect an external handler to step and switch power
void attachStep(handler)
void attachPower(handler)
```

### Example
For more examples see **examples**!
```cpp
#include <GyverStepper.h>
GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);

void setup() {
  Serial.begin(115200);
  // speeding
  stepper.setRunMode(KEEP_SPEED);

  // speeding
  stepper.setSpeed(120);    // stepwise
  stepper.setSpeedDeg(80);  // degrees/sec

  // follow-up
  stepper.setRunMode(FOLLOW_POS);

  // position
  stepper.setTarget(-2024);    // stepwise
  stepper.setTargetDeg(-360);  // degree

  // Maximum velocity setting in degrees/sec
  stepper.setMaxSpeedDeg(400);

  // Maximum speed setting in steps/sec
  stepper.setMaxSpeed(400);

  // acceleration
  stepper.setAccelerationDeg(300);

  // acceleration
  stepper.setAcceleration(300);

  // turn off the motor when the goal is reached
  stepper.autoPower(true);

  // turn on the motor (if pin en is indicated)
  stepper.enable();
}

void loop() {
  // We're just going round and round.
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
### Description
Lightweight GyverStepper
- Easy by a few kB, all integer.
- Increased positioning accuracy
- More efficient hybrid motion algorithm
- Accelerating to target
- Rotating at speed
- Max speed:
  - Normal mode: 37,000 steps / s at full, 18,000 steps / s at acceleration
  - Fast profile: 37,000 steps / s at full, 37,000 steps / s at acceleration
- Optimized for work on timer interruption
- Inherit StepperCore

### Logic of work
- setTarget()/setTargetDeg() sends the motor to the specified position.
- The movement occurs in a tick(), which must be interviewed constantly. Or in tickManual, which must be called with a period derived from getPeriod() and counted at each step.
- Tick() will return true if the engine is rotating
- ready() will return true once if the motor reached the target and stopped
- While moving to the target, you can cause pause(), then the motor will reach the point and stop, ready() will not return true.
- During the movement to the target, you can call stop(), then the motor will brake with a given acceleration, ready() will not return the true
- During the movement to the target, you can call the brake(), then the motor will stop, ready() will not return true.
- After stopping, you can call resume(), the motor will continue to move to the target.
- The constant rotation is setSpeed()/setSpeedDeg(). You can stop abruptly – stop() or brake()
- Speed and acceleration can be set at any time, but they are applied after the engine stops!

<details >>
<summary>Return</summary >>

### Initialization
```cpp
GStepper2<STEPPER2WIRE> stepper(шаговНаОборот, step, dir);                          // step-dir
GStepper2<STEPPER2WIRE> stepper(шаговНаОборот, step, dir, en);                      // Driver step-dir + pin enable
GStepper2<STEPPER4WIRE> stepper(шаговНаОборот, pin1, pin2, pin3, pin4);             // driver
GStepper2<STEPPER4WIRE> stepper(шаговНаОборот, pin1, pin2, pin3, pin4, en);         // driver 4 pin + enable
GStepper2<STEPPER4WIRE_HALF> stepper(шаговНаОборот, pin1, pin2, pin3, pin4);        // driver 4 pin halfway
GStepper2<STEPPER4WIRE_HALF> stepper(шаговНаОборот, pin1, pin2, pin3, pin4, en);    // driver 4 pin halfway + enable

GStepper2<STEPPER2WIRE, STEPPER_VIRTUAL> stepper;    // virtual step-dir driver
GStepper2<STEPPER4WIRE, STEPPER_VIRTUAL> stepper;    // 4-pin virtual driver
```

### Use of use
```cpp
// === Inherited from Stepper ===
void step();                                // step up
void invertEn(bool val);                    // invert
void reverse(bool val);                     // reverse
void disable();                             // switch off power and EN
void enable();                              // power up and EN
void attachStep(void (*handler)(uint8_t));  // plug in
void attachPower(void (*handler)(bool));    // plug in

int32_t pos;                                // stepwise
int8_t dir;                                 // direction (1, -1)

// ========= GStepper2 ==========
// ticker
bool tick();                                // Motion ticker, call frequently. It will return true if the engine is moving.
bool tickManual();                          // manual ticker to call in interrupt timer with period getPeriod(). It will return true if the engine is moving.
bool ready();                               // Once return true, if the engine reached the established position and stopped

// rotation
void setSpeed(int16_t speed);               // set the speed in steps/sec and start the rotation
void setSpeed(float speed);                 // set the speed in steps/sec (float) and start the rotation

// target
void setTarget(int32_t ntar, GS_posType type = ABSOLUTE);       // set the goal in steps and optionally ABSOLUTE/RELATIVE mode
void setTargetDeg(int32_t ntar, GS_posType type = ABSOLUTE);    // set the target in degrees and optionally ABSOLUTE/RELATIVE mode
int32_t getTarget();                                            // Get a Target Position in Steps

void setAcceleration(uint16_t nA);          // acceleration
void setMaxSpeed(int speed);                // set the speed when following the position setTarget() in steps / sec
void setMaxSpeed(float speed);              // set the speed when following the position setTarget() in steps / sec, float
void setMaxSpeedDeg(int speed);             // set the speed when proceeding to the position in hail / sec
void setMaxSpeedDeg(float speed);           // set the speed of movement when proceeding to the position in grad/sec, float

void setCurrent(int32_t npos);              // position
int32_t getCurrent();                       // get the current position
void reset();                               // drop the current position to 0

// anything
void autoPower(bool mode);                  // Auto shutdown of the motor when reaching the position - true (by default. false)
uint32_t getPeriod();                       // get the current tic period
void brake();                               // stop
void pause();                               // Pause - get to a given point and wait (ready() will not return true while you are on pause)
void resume();                              // Continue after a stop/pause
uint8_t getStatus();                        // Current status: 0 - stand, 1 - go, 2 - go to the pause point, 3 - spin at speed, 4 - brake

// ===============
// defile before connecting the library
#define GS_NO_ACCEL                         // disable the acceleration module (reduce code weight)

#define GS_FAST_PROFILE размер_массива (например 10)
Включает быстрый планировщик скорости. Участок разгона/торможения разбивается 
на указанное количество отрезков (+8 байт SRAM на участок), на них скорость будет одинаковая. 
Это позволяет быстро вычислять скорость мотора и достигнуть 30000 шаг/с на участке разгона 
(в обычном режиме в два раза меньше).
```

### Example
For more examples see **examples**!
```cpp
// Let's turn there, tick in the loop.

#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 3);

void setup() {
  Serial.begin(9600);
  //stepper.enable();
  stepper.setMaxSpeed(100);     // speed
  stepper.setAcceleration(200); // acceleration
  stepper.setTarget(300);       // goal
}

bool dir = 1;
void loop() {
  stepper.tick();   // The motor is spinning asynchronously here.

  // if you came
  if (stepper.ready()) {
    dir = !dir;   // unfold
    stepper.setTarget(dir * 300); // go the other way
  }

  // port-out
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
### Description
Multi-axis trajectories planner for stepper motors and CNC machine creation
- Stop at every point. Boofer for one next position
- Max speed:
  - Normal mode: 37,000 steps / s at full, 14,000 steps / s at acceleration
  - Fast profile: 37,000 steps / s at full, 37,000 steps / s at acceleration
- Trapeceidal speed profile (order 2 scheduler)
- Adjusting speed and acceleration
- Any number of axes. They will move synchronously to their goals.
- Fast Integer Trajectory and Speed Planning Model
- Constant rotation mode for one axis (for movement to the tip for example)
- Brake/smooth stop/pause on scheduler trajectory
- Optimized for work on timer interruption
- Quick control of stepping pins for the Arduino AVR
- Speed and acceleration can be set at any time, but they are applied after the engine stops!

### Logic of work
The scheduler manages any number of motors, rotating them to the specified position. In this version
stop occurs at each point of the trajectory, after which the flag is raised ready() and waits
setting the next point.
- See the simulation in Processing: Planner Simulation/Planner folder

<details >>
<summary>Return</summary >>

### Initialization
```cpp
GPlanner<драйвер, количество осей> planner;
```

### Use of use
```cpp
void addStepper(uint8_t axis, Stepper &stp);    // connect the Stepper motor to the axis
// note: the driver type must match the scheduler and motors

void setBacklash(uint8_t axis, uint16_t steps); // set the luft compensation on the axis axis in the number of steps
void enable();                              // motorize
void disable();                             // turn off
void power(bool v);                         // switch off

// STRUCTIONS
void setMaxSpeed(float nV);                 // setting the maximum scheduler speed in step/sec
void setAcceleration(uint16_t nA);          // setting scheduler acceleration in step/sec2

// Planners
uint32_t getPeriod();                       // Returns time to MX until the next tick/tickManual call
bool ready();                               // True - Ready to take the next route point
void pause();                               // Pause (to reach a given point and wait). Ready() will not return true until you pause
void stop();                                // stop smoothly (with a predetermined acceleration)
void brake();                               // Stop motors from any mode
void resume();                              // continue after stopping/pausing
void reset();                               // reset all motor counters to 0
void home();                                // zero off n
uint8_t getStatus();                        // Current status: 0 - stand, 1 - go, 2 - go to the pause point, 3 - spin at speed

// Speed
void setSpeed(uint8_t axis, float speed);   // constant rotation mode for the axis axis at speed step/sec (IB negative)

// Position
void setCurrent(int16_t cur[]);             // position
void setCurrent(int32_t cur[]);             // position
int32_t getCurrent(int axis);               // Get the current position on the axis axis

// Set your goal in steps and start moving. type - ABSOLUTE (by default) or RELATIVE
// ABSOLUTE - specific coordinates of the point where to go
// RELATIVE - displacement relative to current motor positions
// Return true if the target is set. false if the objective is the same as the current one
bool setTarget(int32_t target[]);
bool setTarget(int16_t target[]);
bool setTarget(int32_t target[], type);
bool setTarget(int16_t target[], type);
int32_t getTarget(int axis);                // target in steps on the axis

// Ticker
// Ticker, call as often as possible. It will return true if the engine turns.
// Here steps are taken both for movement on points and for rotation on speed.
bool tick();

// A hand ticker to call in interruption or whatever. Perform 20..50 us.
bool tickManual();

// =================
// announce
#define GS_FAST_PROFILE размер_массива (например 10)
Включает быстрый планировщик скорости. Участок разгона/торможения разбивается 
на указанное количество отрезков (+8 байт SRAM на участок), на них скорость будет одинаковая. 
Это позволяет быстро вычислять скорость мотора и достигнуть 30000 шаг/с на участке разгона 
(в обычном режиме в два раза меньше).
```

### Example
For more examples see **examples**!
```cpp
// Basic Example: How to Create and Run a Planner
// When starting, the engines will be sent to the first position.
// When you get there, the second. After that, the movement will stop.
// Open the plotter and watch the charts.

#include "GyverPlanner.h"
// create Stepper class motors indicating the type of driver and pins
// Motors should have the same type of driver.
// They're beautiful.
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);

// create a scheduler, specify in <> the type of driver as the motors
// number of axles equal to the number of motors (any more than 1)
GPlanner<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(115200);
  // We put steppes on the axles.
  planner.addStepper(0, stepper1);  // zero-axis
  planner.addStepper(1, stepper2);  // axis 1

  // setting acceleration and speed
  planner.setAcceleration(100);
  planner.setMaxSpeed(300);

  planner.reset();  // Reset all positions to 0 (they are already at 0 at launch)

  // array with target axis positions, the size of the array is equal to the number of axes
  int target[] = {300, 200};

  // send out
  planner.setTarget(target);
}

void loop() {
  // This is where the motors move, call as often as possible.
  planner.tick();

  // It will return true if all the engines are gone.
  if (planner.ready()) {
    // load up a new point
    int newTarget[] = {10, 50};
    planner.setTarget(newTarget);
  }

  // Asynchronously display in the graphic port
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
### Description
Multi-axis trajectories planner for stepper motors and CNC machine creation
- Speed planning on the march. TRAINING BUFFER
- Max speed:
  - Normal mode: 37,000 steps / s at full, 14,000 steps / s at acceleration
  - Fast profile: 37,000 steps / s at full, 37,000 steps / s at acceleration
- Trapeceidal speed profile (order 2 scheduler)
- Adjusting speed and acceleration
- Any number of axes. They will move synchronously to their goals.
- Fast Integer Trajectory and Speed Planning Model
- Constant rotation mode for one axis (for movement to the tip for example)
- Brake/smooth stop/pause on scheduler trajectory
- Optimized for work on timer interruption
- Quick control of stepping pins for the Arduino AVR
- Speed and acceleration can be set at any time, but they are applied after the engine stops!

### Logic of work
The scheduler manages any number of motors, rotating them to the specified position. In this version
A trajectory buffer is implemented, which can be filled with points until available() returns true.
addTarget() receives:
- Array of points specified at initialization of size
- Stop flag. If you transmit 1 - the scheduler will stop the engine at this point and will wait for the further command resume()
- Point type: ABSOLUTE (absolute coordinate) or RELATIVE (relative to the previous point)

When the plane operator arrives at the stop point, he pauses (for example, to turn off the instrument),
After performing the necessary actions, call resume() and it continues to move.
Unlike the previous GPlanner, GPlanner2 implements buffer path calculation and planning.
speeds for all points, allowing the system to move faster and not brake at every point.
- See the simulation in Processing: Planner Simulation/Planner2 folder

<details >>
<summary>Return</summary >>

### Initialization
```cpp
GPlanner2<драйвер, количество осей> planner;                 // declaration
GPlanner2<драйвер, количество осей, размер буфера> planner;  // + buffer size (by omission 32)
```

### Use of use
```cpp
void addStepper(uint8_t axis, Stepper &stp);    // connect the Stepper motor to the axis
// note: the driver type must match the scheduler and motors

void setBacklash(uint8_t axis, uint16_t steps); // set the luft compensation on the axis axis in the number of steps
void enable();                              // motorize
void disable();                             // turn off
void power(bool v);                         // switch off

// STRUCTIONS
void setMaxSpeed(float nV);                 // setting the maximum scheduler speed in step/sec
void setAcceleration(uint16_t nA);          // setting scheduler acceleration in step/sec2
void setDtA(float newDta);                  // set the change of speed in turn, 0.0... 1.0 in silence. 0.3.3

// Planners
uint32_t getPeriod();                       // Returns time to MX until the next tick/tickManual call
void start();                               // start
void stop();                                // stop smoothly (with a predetermined acceleration)
void brake();                               // Stop motors from any mode
void resume();                              // Continue after a stop or end point of the route
void reset();                               // reset all motor counters to 0
bool ready();                               // flag to reach the stopping point. After that, you need to call for a resume.
bool available();                           // True - in the planner buffer there is a place under the new point

uint8_t getStatus();                        // Current status:
// 0 Team waiting (suspended)
// 1 wait for a buffer
// 2 on the way
// 3 on pause
// 4 on stop
// 5 spins setSpeed

// Speed
void setSpeed(uint8_t axis, float speed);   // constant rotation mode for the axis axis at speed step/sec (IB negative)

// Position
// Add a new route point. Array of coordinates, ending flag and absolute/relative
void addTarget(int32_t tar[], uint8_t l, GS_posType type = ABSOLUTE);
void addTarget(int16_t tar[], uint8_t l, GS_posType type = ABSOLUTE);
// ABSOLUTE - specific coordinates of the point where to go
// RELATIVE - displacement relative to current motor positions

void setCurrent(int16_t cur[]);             // position
void setCurrent(int32_t cur[]);             // position
int32_t getCurrent(int axis);               // Get the current position on the axis axis
int32_t getTarget(int axis);                // Get the current goal in steps on the axis axis

// Ticker
// Ticker, call as often as possible. It will return true if the engine turns.
// Here steps are taken to move on points, to rotate on speed, as well as the restructuring of the buffer.
bool tick();

// A hand ticker to call in interruption or whatever. Perform 20..50 us.
bool tickManual();

// buffer handler. He's called in the tick. You need to call manually when working with TickManual
// will return true if the scheduler has sent the motors to a new position (at which point you can start the timer)
void checkBuffer();

// =================
// announce
#define GS_FAST_PROFILE размер_массива (например 10)
Включает быстрый планировщик скорости. Участок разгона/торможения разбивается 
на указанное количество отрезков (+8 байт SRAM на участок), на них скорость будет одинаковая. 
Это позволяет быстро вычислять скорость мотора и достигнуть 30000 шаг/с на участке разгона 
(в обычном режиме в два раза меньше).
```

### Example
For more examples see **examples**!
```cpp
// notebook
// Take a look at the schedule and start stepperPlot.

int path[][2] = {
  {100, 250},
  {160, 30},
  {230, 250},
  {60, 100},
  {270, 100},
};

// Number of points (let the compiler count)
// as the weight of the entire array / (2+2) byte
int nodeAmount = sizeof(path) / 4;

#include "GyverPlanner2.h"
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);
GPlanner2<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(115200);
  // We put steppes on the axles.
  planner.addStepper(0, stepper1);  // zero-axis
  planner.addStepper(1, stepper2);  // axis 1

  // setting acceleration and speed
  planner.setAcceleration(500);
  planner.setMaxSpeed(500);

  // The starting point of the system must coincide with the first point of the route.
  planner.setCurrent(path[0]);
  planner.start();
}

int count = 0;  // router
void loop() {
  // This is where the motors move, call as often as possible.
  planner.tick();

  // if there is room in the planner buffer
  if (planner.available()) {
    // Add a route point and whether it is a stop point (0 - no)
    planner.addTarget(path[count], 0);
    if (++count >= sizeof(path) / 4) count = 0; // loop
  }

  // Asynchronously display in the graphic port
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
## Versions
- v1.1 - added smooth speed control in KEEP SPEED (see example accelDeccelButton)
- v1.2 - ESP8266 support added
- v1.3 - changed the logic of setTarget(, RELATIVE)
- v1.4 - added delay for STEP, you can configure DRIVER STEP TIME
- v1.5 - Fixed bug for ESP boards
- v1.6 - Fixed stop for STEPPER4WIRE HALF, speed can be set in float (for slow speeds)
- v1.7 - Fixed a negative speed bug (thanks Evgeny Solodova)
- v1.8 - KEEP SPEED mode fixed
- v1.9 - Fixed error with esp function max
- v1.10 - increased accuracy
- v1.11 - increased accuracy of speed setting
- v1.12 - Fixed smooth operation in KEEP SPEED. Support for "external" drivers is added. Removed SMOOTH argument from setSpeed
- v1.13 - fix small bugs, optimization
- v1.14 - Acceleration and braking errors fixed in KEEP SPEED
- v1.15 - Optimization, minor bugs fixed, stop() no longer resets maxSpeed
- v1.15.2 - added EN activation if specified, even when autoPower is disabled
- v2.0 is optimization. The core of the stepper is placed in a separate Stepper class. Multi-axis trajectory planners added
- v2.1 added GyverStepper2, a simplified and optimized version of GyverStepper
- v2.1.1 - Fixed bug in GyverStepper
- v2.1.2 - Digispark compatibility
- v2.1.3 – Fixed FOLLOW POS in GStepper, fixed RELATIVE in GPlanner2, and fixed the jerk bug
- v2.1.4 - GPlanner2: corrected jerks, added adaptive trajectory rearrangement without stopping, slightly optimized calculations
- v2.1.5 - the ability to change speed and acceleration during operation of the scheduler (GStepper2, GPlanner, GPlanner2)
- v2.1.6 - Fixed compilation error when calling disable() in GStepper
- v2.1.7 - added clearBuffer() to GPlanner2
- v2.1.8 - Optimization, fixed KEEP SPEED in GStepper
- v2.2.0 - GS FAST PROFILE speed profile added for GStepper2, GPlanner, GPlanner2. Support for tracking mode for GStepper2
- v2.2.1 - small optimization of SRAM
- v2.3 - fix compiler warnings, support for esp32
- v2.4 - increased smoothness of movement of steppes in Planner and Planner2. Fixed bug in Stepper2
- v2.5 - Smooth speed change fixed for KEEP SPEED
- v2.6
    - disable() in virtual mode disables the signal from the motor (for 4-wire drivers)
    - Improved performance for step-dir drivers
    - Add autoPower() to GStepper2
    - Corrected jerk when changing direction in GStepper
- v2.6.1 - Corrected bug in GStepper2
- v2.6.2 - Optimized calculations in GStepper2, GPlanner and GPlanner2
- v2.6.3 - reverse() in step-dir driver now applies immediately
- v2.6.4 - fixed RELATIVE setTarget() in GPlannerhttps://github.com/GyverLibs/GyverStepper/pull/11
- v2.7
  - Fixed various compilation errors
  - Fixed some critical bugs
  - Added food management to planners
  - Added backup compensation to planners
  - Zero Acceleration Bugs Fixed in All Libraries
  - corrected slow stop and impact at high acceleration
  - Increased performance for esp8266

<a id="feedback"></a>
## Bugs and feedback
If you find bugs, create **Issue**, or better write to the mail immediately.[alex@alexgyver.ru](mailto:alex@alexgyver.ru)
The library is open for revision and your **Pull Requests*!

When reporting bugs or incorrect work of the library, it is necessary to specify:
- Library version
- What is used by the IC
- SDK version (for ESP)
- Arduino IDE version
- Are embedded examples that use features and designs that cause bugs in your code working correctly?
- What code was downloaded, what work was expected from it and how it works in reality
- Ideally, attach the minimum code in which the bug is observed. Not a canvas of a thousand lines, but a minimum code.

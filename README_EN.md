This is an automatic translation, may be incorrect in some places. See sources and examples!

# GyverStepper
A powerful library for controlling stepper motors with Arduino
- Support for 4 pin (step and half step) and STEP-DIR drivers
- Automatic power off when the target is reached
- Operating modes:
    - Rotation at a given speed. Smooth acceleration and deceleration with acceleration
    - Following to the position with acceleration and speed limit
    - Following to the position with the given speed (without acceleration)
- Fast step control algorithm
- Support for "virtual" drivers
- Built-in multi-axis path planner

### Compatibility
Compatible with all Arduino platforms (using Arduino functions)

### Documentation
The library has [extended documentation](https://alexgyver.ru/GyverStepper/)

## Content
- [Install](#install)
- [StepperCore](#core)
- [GyverStepper](#stepper)
- [GyverStepper2](#stepper2)
- [GyverPlanner](#planner)
- [GyverPlanner2](#planner2)
- [Versions](#versions)
- [Bugs and feedback](#feedback)


## Ahhh why so many things?!
The library contains a set of tools for different scenarios of working with stepper motors
- *StepperCore.h* [class **Stepper**]: the core of all other classes, can quickly click pins (AVR) and take one step for the configured driver type. Supports 4 phase step/half step as well as step-dir drivers.
- *GyverStepper.h* [class **GStepper**]: main heavy library, many settings. One m movementotor with acceleration to a given position or rotation at a given speed. Not very optimal performance in timer interrupt.
- *GyverStepper2.h* [class **GStepper2**]: new lightweight version of GyverStepper, almost completely compatible with it. More optimal integer hybrid acceleration motion algorithm, lighter weight. Optimized to work in a timer interrupt.
- *GyverPlanner.h* [class **GPlanner**]: multi-axis trajectory planner, motion with acceleration (2nd order). Stop at every point. Optimum performance in timer interrupt.
- *GyverPlanner2.h* [class **GPlanner2**]: multi-axis trajectory planner, motion with acceleration (2nd order). Speed ​​planning on the route, optimal movement by points. Optimum performance in timer interrupt.

<a id="install"></a>
## Installation
- The library can be found by the name **GyverStepper** and installed through the library manager in:
    - Arduino IDE
    - Arduino IDE v2
    - PlatformIO
- [Download library](https://github.com/GyverLibs/GyverStepper/archive/refs/heads/main.zip) .zip archive for manual installation:
    - Unzip and put in *C:\Program Files (x86)\Arduino\libraries* (Windows x64)
    - Unzip and put in *C:\Program Files\Arduino\libraries* (Windows x32)
    - Unpack and put in *Documents/Arduino/libraries/*
    - (Arduino IDE) automatic installation from .zip: *Sketch/Include library/Add .ZIP library…* and specify the downloaded archive
- Read more detailed instructions for installing libraries [here] (https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE% D0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)

<a id="core"></a>
## StepperCore
### Description
The core of the library for controlling stepper motors:
- 4 phase and STEP DIR drivers
- EN pin support
- Virtual Driver
- Fast IO algorithm for AVR

<details>
<summary>Expand</summary>

### Initialization (StepperCore)
```cpp
Stepper<STEPPER2WIRE> stepper(step, dir);// step-dir driver
Stepper<STEPPER2WIRE> stepper(step, dir, en); // driver step-dir + enable pin
Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4); // driver 4 pin
Stepper<STEPPER4WIRE> stepper(pin1, pin2, pin3, pin4, en); // driver 4 pin + enable
Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4); // driver 4 pin half step
Stepper<STEPPER4WIRE_HALF> stepper(pin1, pin2, pin3, pin4, en); // driver 4 pin half step + enable

Stepper<STEPPER2WIRE, STEPPER_VIRTUAL> stepper; // virtual driver step-dir
Stepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper; // virtual driver 4 pin
```

### Usage (StepperCore)
```cpp
voidstep(); // make a step
void invertEn(bool val); // invert the behavior of the EN pin
void reverse(bool value); // invert motor direction
void disable(); // turn off power and EN
void enable(); // power on and EN
void attachStep(void (*handler)(uint8_t)); // connect step handler
void attachPower(void (*handler)(bool)); // connect power handler

int32_t pos; // current position in steps
int8_t dir; // direction (1, -1)
```

### Example
See **examples** for other examples!
```cpp
#include <StepperCore.h>
Stepper<STEPPER2WIRE> stepper(2, 3);

void setup() {
  stepper.dir = 1; // or -1
  stepper.pos = 0; // position access
}

void loop() {
  // twist manually
  stepper.step(); // make a step
  delay(10);
}
```
</details>

<a id="stepper"></a>
## GyverStepper and GyverStepper2
- GStepper2 much lighter and stronger optimized
- GStepper2 is more intended for movement along trajectory points, but the "following" mode also works in it
- GStepper2 optimized to work in timer interrupt
- GStepper2 uses smooth motor movement algorithm by default
- GStepper2 is likely to be slightlyCranberries load the microcontroller more during operation than GStepper. But much less than GStepper in SMOOTH_ALGORITHM mode

### How GStepper works
- Fast algorithm (default): there are two timers in the library: a step timer (different period) and a scheduler timer (10-30 milliseconds). According to the scheduler timer, the trajectory is recalculated and the direction of movement and the current required speed are calculated. From this speed, a new period for the step timer is obtained.
- Smooth algorithm (SMOOTH_ALGORITHM setting): one step timer works, at each step the movement speed and time to the next step are recalculated.

### How GStepper2 works
- Smooth profile (default): it uses the SMOOTH_ALGORITHM smooth motion algorithm like in GStepper, but optimized and speeded up by 2-3 times. All the heavy trajectory calculations are done by setting a new target position in setTarget. Further on the step timer, the motor steps and quickly calculates a new time for the next step. This allows a minimum of time to be wasted in the timer interrupt, if one is used.
- Fast profile (setting GS_FAST_PROFILE): everything is the same, but in the step timer there is no calculation, but a period selection from a pre-calculated table, which allows you to move with acceleration up to speeds of 30,000 steps per second.

## GyverStepper
### Description
Library for controlling stepper motors with Arduino
- Support for 4 pin (step and half step) and STEP-DIR drivers
- Automatic power off when the target is reached
- Operating modes:
    - Rotation at a given speed. Smooth acceleration and deceleration with acceleration
    - Following to the position with acceleration and speed limit
    - Following to the position with the given speed (without acceleration)

<details>
<summary>Expand</summary>

### Initialization
```cpp
// steps - steps per shaft revolution (for calculations with degrees)
// step, dir, pin1, pin2, pin3, pin4 - any GPIO
// en - driver disable pin, any GPIO
GStepper<STEPPER2WIRE> stepper(steps, step, dir); // step-dir driver
GStepper<STEPPER2WIRE> stepper(steps, step, dir, en); // driver step-dir + enable pin
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4); // driver 4 pin
GStepper<STEPPER4WIRE> stepper(steps, pin1, pin2, pin3, pin4, en); // driver 4 pin + enable
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4); // driver 4 pin half step
GStepper<STEPPER4WIRE_HALF> stepper(steps, pin1, pin2, pin3, pin4, en); // driver 4 pin half step + enable
GStepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper(2048); // virtual driver, specify only the number of steps
```

### Usage
```cpp
// Note: hereinafter, "default" means "even if the function is not called"

// This is where the motor moves, call as often as possible!
// Has a built in timer
// Returns true if the motor is moving towards the target or spinning at KEEP_SPEED
bool tick();

// Returns the same as tick, i.e. motor spinning or not
bool getState();

// Invert motor direction - true (default false)
void reverse(bool dir);

// invert EN pin behavior - true (default false)
void invertEn(bool rev);

// Set the operating mode, mode:
// FOLLOW_POS - following to the position setTarget(...)
// KEEP_SPEED - keep speed setSpeed(...)
void setRunMode(GS_runMode);

// Set the current position of the motor in steps and degrees
void setCurrent(long pos);
void setCurrentDeg(float pos);

// Read current motor position in steps and degrees
long getCurrent();
float getCurrentDeg();

// set target position in steps and degrees (for FOLLOW_POS mode)
// type - ABSOLUTE or RELATIVE, default is ABSOLUTE
void setTarget(long pos);
void setTarget(long pos, GS_posType type);
void setTargetDeg(float pos);
void setTargetDeg(float pos, GS_posType type);

// Get the target position in steps and degrees
long getTarget();
float getTargetDeg();

// Set the maximumSpeed ​​cranberry (modulo) in steps/second and degrees/second (for FOLLOW_POS mode)
// by default 300
void setMaxSpeed(float speed);
void setMaxSpeedDeg(float speed);

// Set acceleration in steps and degrees per second (for FOLLOW_POS mode).
// If set to 0, acceleration is disabled and the motor is running
// according to the constant maximum speed profile setMaxSpeed().
// Default 300
void setAcceleration(int accel);
void setAccelerationDeg(float accel);

// Auto turn off EN when position is reached - true (default false)
void autoPower(bool mode);

// Soft stop with a given acceleration
void stop();

// hard stop
void brake();

// Hard stop + reset position to 0 (for limit switches)
void reset();

// Set target speed in steps/second and degrees/second (for KEEP_SPEED mode)
void setSpeed(float speed);
void setSpeedDeg(float speed);

// Get target speed in steps/second and degrees/second (for KEEP_SPEED mode)
float getSpeed();
float getSpeedDeg();

// Turn on the motor (pin EN)
void enable();

// Turn off the motor (pin EN)
void disable();

// Returns the minimum tick period of the motor in microseconds at the setMaxSpeed() speed.
// Can be used to set up timer interrupts, in the handler of which tick () will lie (see the timerISR example)
uint16_t getMinPeriod();

// Current "tick" period for debugging and stuff
uint16_tstepTime;

// connect external handler for step and power switch
void attachStep(handler)
void attachPower(handler)
```

### Example
See **examples** for other examples!
```cpp
#include <GyverStepper.h>
GStepper<STEPPER4WIRE> stepper(2048, 5, 3, 4, 2);

void setup() {
  Serial.begin(115200);
  // speed maintenance mode
  stepper.setRunMode(KEEP_SPEED);

  // you can set the speed
  stepper.setSpeed(120); // in steps/sec
  stepper.setSpeedDeg(80); // in degrees/sec

  // mode of following to the target position
  stepper.setRunMode(FOLLOW_POS);

  // you can set the position
  stepper.setTarget(-2024); // in steps
  stepper.setTargetDeg(-360); // in degrees

  // set max. speed in degrees/sec
  stepper.setMaxSpeedDeg(400);

  // set max. speed in steps/sec
  stepper.setMaxSpeed(400);

  // set acceleration in degrees/sec/sec
  stepper.setAccelerationDeg(300);

  // set acceleration in steps/sec/sec
  stepper.setAcceleration(300);

  // turn off the motor when the target is reached
  stepper.autoPower(true);

  // turn on the motor (if pin en is specified)
  stepper.enable();
}

void loop() {
  // just twist back and forth
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
- A few kB lighter, all integer
- Improved positioning accuracy
- More efficient hybrid movement algorithm
- Moving towards the target with acceleration
- Rotate with speed
- Max. speed:
  - Normal mode: 37000 steps/s at full, 18000 steps/s at acceleration
  - Fast profile: 37,000 strides/s at full speed, 37,000 strides/s during acceleration
- Optimized to work on timer interrupt
- Inherits StepperCore

### Operation logic
- setTarget()/setTargetDeg() sends the motor to the specified position
- Movement occurs in tick(), which must be polled constantly. Or in tickManual, which needs to be called with the period obtained from getPeriod() and recalculated at each step
- tick() will return true if the motor is spinning
- ready() will return true once if the motor has reached the target and stopped
- While moving towards the target, you can call pause(), then the motor will reach the point and stop, ready() will not return true
- While moving towards the target, you can call stop(), then the motor will slow down with a given acceleration, ready() will not return true
- While moving towards the target, you can call brake(), then the motor will stop, ready() will not return true
- After stopping, you can call resume(), the motor will continue moving towards the target
- Constant rotation is set by setSpeed()/setSpeedDeg(). You can stop abruptly - stop() or brake()
- Speed ​​and acceleration can be set at any time, but they are applied after the motor stops!

<details>
<summary>Expand</summary>

### Initialization
```cpp
GStepper2<STEPPER2WIRE> stepper(stepsPerTurn, step, dir); // step-dir driver
GStepper2<STEPPER2WIRE> stepper(stepsPerTurn, step, dir, en); // driver step-dir + enable pin
GStepper2<STEPPER4WIRE> stepper(stepsPrev, pin1, pin2, pin3, pin4); // driver 4 pin
GStepper2<STEPPER4WIRE> stepper(stepsPerTurn, pin1, pin2, pin3, pin4, en); // driver 4 pin + enable
GStepper2<STEPPER4WIRE_HALF> stepper(stepsPerTurn, pin1, pin2, pin3, pin4); // driver 4 pin half step
GStepper2<STEPPER4WIRE_HALF> stepper(stepsPerTurn, pin1, pin2, pin3, pin4, en); // driver 4 pin half step + enable

GStepper2<STEPPER2WIRE, STEPPER_VIRTUAL> stepper; // virtual driver step-dir
GStepper2<STEPPER4WIRE, STEPPER_VIRTUAL> stepper; // virtual driver 4 pin
```

### Usage
```cpp
// === inherited from Stepper ====
voidstep(); // make a step
void invertEn(bool val); // invert the behavior of the EN pin
void reverse(bool value); // invert motor direction
void disable(); // turn off power and EN
void enable(); // power on and EN
void attachStep(void (*handler)(uint8_t)); // connect step handler
void attachPower(void (*handler)(bool)); // connect power handler

int32_t pos; // current position in steps
int8_t dir; // direction (1, -1)

// ========= GStepper2 ==========
// ticker
bool tick(); // movement ticker, call frequently. Returns true if the motor is moving
bool tickManual(); // manual ticker to call in interrupt timeera with getPeriod() period. Returns true if the motor is moving
bool ready(); // returns true once if the motor has reached the set position and stopped

// rotation
void setSpeed(int16_t speed); // set speed in steps/sec and start rotation
void setSpeed(float speed); // set speed in steps/sec (float) and start rotation

// moving towards the target
void setTarget(int32_t ntar, GS_posType type = ABSOLUTE); // set target in steps and optional ABSOLUTE/RELATIVE mode
void setTargetDeg(int32_t ntar, GS_posType type = ABSOLUTE); // set target in degrees and optional ABSOLUTE/RELATIVE mode
int32_t getTarget(); // get target position in steps

void setAcceleration(uint16_t nA); // set acceleration in steps/sec^2
void setMaxSpeed(int speed); // set the movement speed when following the position setTarget() in steps/sec
void setMaxSpeed(float speed); // set the movement speed when following the position setTarget() in steps/sec, float
void setMaxSpeedDeg(int speed); // set the speed of movement when following the position in deg/sec
void setMaxSpeedDeg(float speed); // set the speed of movement when following the position in deg / sec, float

void setCurrent(int32_tnpos); // set current position
int32_t getCurrent(); // get current position
void reset(); // reset current position to 0

// anything
void autoPower(bool mode); // auto-shutdown of the motor when the position is reached - true (default false)
uint32_t getPeriod(); // get the current tick period
void brake(); // abruptly stop the motor
void pause(); // pause - get to the given point and wait (ready() will not return true while you are paused)
void resume(); // continue moving after stop/pause
uint8_t getStatus(); // current status: 0 - stop, 1 - drive, 2 - drive to the pause point, 3 - spin at speed, 4 - slow down

// ===== DEFINE SETTINGS =====
// define before linking the library
#define GS_NO_ACCEL // disable acceleration movement module (reduce code weight)

#define GS_FAST_PROFILE array_size (e.g. 10)
Includes a fast speed scheduler. The acceleration/deceleration section is broken
for the specified number of segments (+8 bytes of SRAM per segment), they will have the same speed.
This allows you to quickly calculate the speed of the motor and reach 30000 steps / s in the acceleration section.
(in normal mode, two times less).
```

### Example
See **examples** for other examples!
```cpp
// spin back and forth, tick in loop

#include "GyverStepper2.h"
GStepper2<STEPPER2WIRE> stepper(2048, 2, 3);

void setup() {
  Serial.begin(9600);
  //stepper.enable();
  stepper.setMaxSpeed(100); // movement speed to the target
  stepper.setAcceleration(200); // acceleration
  stepper.setTarget(300); // goal
}

bool dir = 1;
void loop() {
  stepper.tick(); // motor spins asynchronously here

  // if arrived
  if (stepper.ready()) {
    dir = !dir; // expand
    stepper.setTarget(dir * 300); // go the other way
  }

  // asynchronous output to the port
  static uint32_t tmr;
  if (millis() - tmr >= 30) {
    tmr = millis();
    serial.println(stepper.pos);
  }
}
```
</details>


<a id="planner"></a>
## GyverPlanner
### Description
Multi-axis path planner for stepper motors and CNC machine tool creation
- STOP AT EVERY POINT. BUFFER FOR ONE NEXT POSITION
- Max. speed:
  - Normal mode: 37000 steps/s at full, 14000 steps/s at acceleration
  - Fast profile: 37,000 strides/s at full speed, 37,000 strides/s during acceleration
- Trapezoidal velocity profile (2nd order planner)
- Setting speed and acceleration
- Any number of axles. There will be twoCranberries move synchronously to the set goals
- Fast integer trajectory and velocity planning model
- Constant rotation mode for one axis (for movement to the limit switch for example)
- Brake / soft stop / pause on the trajectory of the planner
- Optimized to work on timer interrupt
- Quick stepper pin control for Arduino AVR
- Speed ​​and acceleration can be set at any time, but they are applied after the motor stops!

### Operation logic
The scheduler controls any number of motors, rotating them to a specified position. In this version
stop occurs at each point of the trajectory, after which the ready () flag is raised and waits
setting the next point.
- See the simulation in Processing: folder Planner Simulation/Planner

<details>
<summary>Expand</summary>

### Initialization
```cpp
GPlanner<driver, number of axes> planner;
```

### Usage
```cpp
void addStepper(uint8_t axis, Stepper &stp); // connect the Stepper class motor to the axis
// note: driver type must match scheduler and motors

// SETTINGS
void setMaxSpeed(float nV); // set the maximum speed of the scheduler in steps / sec
void setAcceleration(uint16_t nA); // setting the scheduler acceleration in steps/sec^2

// SCHEDULER
uint32_t getPeriod(); // returns the time in µs until the next call to tick/tickManual
bool ready(); // true - ready to accept the next waypoint
void pause(); // pause (get to the given point and wait). ready() will not return true while you are paused
void stop(); // stop smoothly (with a given acceleration)
void brake(); // abruptly stop the motors from any mode
void resume(); // continue after stop/pause
void reset(); // reset counters of all motors to 0
uint8_t getStatus(); // current status: 0- we stand, 1 - we go, 2 - we go to the pause point, 3 - we spin at speed

// SPEED
void setSpeed(uint8_t axis, float speed); // constant rotation mode for axis with speed step/sec (can be negative)

// POSITION
void setCurrent(int16_t cur[]); // set the current position of the motors
void setCurrent(int32_t cur[]); // set the current position of the motors
int32_t getCurrent(int axis); // get the current position along the axis

// set the goal in steps and start moving. type - ABSOLUTE (default) or RELATIVE
// ABSOLUTE - specific coordinates of the point where to move
// RELATIVE - offset relative to the current positions of the motors
// will return true if the target is set. false if the target is the same as the current one
bool setTarget(int32_t target[]);
bool setTarget(int16_t target[]);
bool setTarget(int32_t target[], type);
bool setTarget(int16_t target[], type);
int32_t getTarget(int axis); // get the goal in steps on the axis

// TICKER
// ticker, call as often as possible. Returns true if the motor is spinning
// steps are taken here for both movement by points and rotation by speed
bool tick();

// manual ticker to call in interrupt or elsewhere. Running 20..50 us
bool tickManual();

// ======= DEFINE SETTINGS =======
// declare before linking the library
#define GS_FAST_PROFILE array_size (e.g. 10)
Includes a fast speed scheduler. The acceleration/deceleration section is broken
for the specified number of segments (+8 bytes of SRAM per segment), they will have the same speed.
This allows you to quickly calculate the speed of the motor and reach 30000 steps / s in the acceleration section.
(in normal mode, two times less).
```

### Example
See **examples** for other examples!
```cpp
// basic example: how to create and run a scheduler
// when starting, the motors will be sent to the first position
// upon reaching - to the second. After that the movement will stop.
// open plotter and see graphs

#include "GyverPlanner.h"
// create stepper class motors with driver type and pins
// MOTORS MUST BE WITH THE SAME DRIVER TYPE
// here they are handsome
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);

// create a scheduler, specify in <> the driver type LIKE MOTORS
// and the number of axles equal to the number of motors (any greater than 1)
GPlanner<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(115200);
  // add steppers on the axis
  planner.addStepper(0, stepper1); // axis 0
  planner.addStepper(1, stepper2); // axis 1

  // set acceleration and speed
  planner.setAcceleration(100);
  planner.setMaxSpeed(300);

  planner reset(); // reset all positions to 0 (they are already at 0 at startup)

  // array with the target positions of the axes, the size of the array is equal to the number of axes
  int target[] = {300, 200};

  // send
  planner.setTarget(target);
}

void loop() {
  // motors move here, call as often as possible
  planner.tick();

  // returns true if all motors have arrived
  if (planner.ready()) {
    // load new point
    int newTarget[] = {10, 50};
    planner.setTarget(newTarget);
  }

  // asynchronously output to the graphics port
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(stepper1.pos);
    Serial print(',');
    Serial.println(stepper2.pos);
  }
}
```
</details>

<a id="planner2"></a>
## GyverPlanner2
### Description
Multi-axis path planner for stepper motors and CNC machine tool creation
- ROUTE SPEED PLANNING. CUSTOMIZED BUFFER
- Max. speed:
  - Normal mode: 37000 steps/s at full, 14000 steps/s at acceleration
  - Fast profile: 37,000 strides/s at full speed, 37,000 strides/s during acceleration
- Trapezoidal velocity profile (2nd order planner)
- Setting speed and acceleration
- Any number of axles. Will move synchronously to the set goals
- Fast integer trajectory and velocity planning model
- Constant rotation mode for one axis (for movement to the endku for example)
- Brake / soft stop / pause on the trajectory of the planner
- Optimized to work on timer interrupt
- Quick stepper pin control for Arduino AVR
- Speed ​​and acceleration can be set at any time, but they are applied after the motor stops!

### Operation logic
The scheduler controls any number of motors, rotating them to a specified position. In this version
implemented a trajectory buffer that can be filled with points as long as available() returns true.
addTarget() accepts:
- An array of points of the size specified during initialization
- Stop flag. If you pass 1, the scheduler will stop the motor at this point and wait for the next resume() command
- Point type: ABSOLUTE (absolute coordinate) or RELATIVE (relative to the previous point)

When the scheduler arrives at the stop point, it pauses (for example, to turn the tool off),
after performing the necessary actions, we call resume () and it continues to move.
Unlike the previous GPlanner, GPlanner2 implements trajectory calculation in the buffer and planning
speed for all points, which allows the system to move faster and not slow down at each point.
- See the simulation in Processing: folder Planner Simulation/Planner2

<details>
<summary>Expand</summary>

### Initialization
```cpp
GPlanner2<driver, number of axes> planner; // declaration
GPlanner2<driver, number of axes, buffer size> planner; // + buffer size (default 32)
```

### Usage
```cpp
void addStepper(uint8_t axis, Stepper &stp); // connect the Stepper class motor to the axis
// note: driver type must match scheduler and motors

// SETTINGS
void setMaxSpeed(float nV); // set the maximum speed of the scheduler in steps / sec
void setAcceleration(uint16_t nA); // setting the scheduler acceleration in steps/sec^2
void setDtA(float newDta); // set dt of speed change in turn, 0.0.. 1.0 by default 0.3

// SCHEDULER
uint32_tgetPeriod(); // returns the time in µs until the next call to tick/tickManual
void start(); // start work
void stop(); // stop smoothly (with a given acceleration)
void brake(); // abruptly stop the motors from any mode
void resume(); // continue after a stop or end point of the route
void reset(); // reset counters of all motors to 0
bool ready(); // flag for reaching the breakpoint. After it, you need to call resume
bool available(); // true - there is room in the scheduler buffer for a new point

uint8_t getStatus(); // current status:
// 0 waiting for command (stopped)
// 1 buffer wait
// 2 on the way
// 3 per pause
// 4 per stop
// 5 is spinning setSpeed

// SPEED
void setSpeed(uint8_t axis, float speed); // constant rotation mode for axis with speed step/sec (can be negative)

// POSITION
// add a new waypoint. Array of coordinates, end flag and absolute/relative
void addTarget(int32_t tar[], uint8_t l, GS_posType type = ABSOLUTE);
void addTarget(int16_t tar[], uint8_t l, GS_posType type = ABSOLUTE);
// ABSOLUTE - specific coordinates of the point where to move
// RELATIVE - offset relative to the current positions of the motors

void setCurrent(int16_t cur[]); // set the current position of the motors
void setCurrent(int32_t cur[]); // set the current position of the motors
int32_t getCurrent(int axis); // get the current position along the axis
int32_t getTarget(int axis); // get the current target in steps on the axis

// TICKER
// ticker, call as often as possible. Returns true if the motor is spinning
// steps are taken here for movement by points, for rotation by speed, as well as buffer rebuilding
bool tick();

// manual ticker to call in breaksor somewhere else. Running 20..50 us
bool tickManual();

// buffer handler. Itself is called in tick. Must be called manually when working with tickManual
// will return true if the scheduler sent the motors to a new position (at this point, you can start the timer)
void checkBuffer();

// ======= DEFINE SETTINGS =======
// declare before linking the library
#define GS_FAST_PROFILE array_size (e.g. 10)
Includes a fast speed scheduler. The acceleration/deceleration section is broken
for the specified number of segments (+8 bytes of SRAM per segment), they will have the same speed.
This allows you to quickly calculate the speed of the motor and reach 30000 steps / s in the acceleration section.
(in normal mode, two times less).
```

### Example
See **examples** for other examples!
```cpp
// example with route stored in memory
// see the graph, or better run stepperPlot

int path[][2] = {
  {100, 250},
  {160, 30},
  {230, 250},
  {60, 100},
  {270, 100},
};

// number of points (let the compiler calculate it for itself)
// as the weight of the entire array / (2+2) bytes
int nodeAmount = sizeof(path) / 4;

#include "GyverPlanner2.h"
Stepper<STEPPER2WIRE> stepper1(2, 3);
Stepper<STEPPER2WIRE> stepper2(4, 5);
GPlanner2<STEPPER2WIRE, 2> planner;

void setup() {
  Serial.begin(115200);
  // add steppers on the axis
  planner.addStepper(0, stepper1); // axis 0
  planner.addStepper(1, stepper2); // axis 1

  // set acceleration and speed
  planner.setAcceleration(500);
  planner.setMaxSpeed(500);

  // the starting point of the system must be the same as the first point of the route
  planner.setCurrent(path[0]);
  planner.start();
}

int count = 0; // waypoint counter
void loop() {
  // motors move here, call as often as possible
  planner.tick();

  // if there is room in the schedule buffer
  if (planner.available()) {
    // add a route point and whether it is a stop point (0 - no)
    planner.addTarget(path[count], 0);
    if (++count >= sizeof(path) / 4) count = 0; // loopscranberries
  }

  // asynchronously output to the graphics port
  static uint32_t tmr;
  if (millis() - tmr >= 20) {
    tmr = millis();
    Serial.print(stepper1.pos);
    Serial print(',');
    Serial.println(stepper2.pos);
  }
}
```
</details>

<a id="versions"></a>
## Versions
- v1.1 - added the ability to smoothly control the speed in KEEP_SPEED (see accelDeccelButton example)
- v1.2 - added support for ESP8266
- v1.3 - setTarget(, RELATIVE) logic changed
- v1.4 - added delay for STEP, can be configured by define DRIVER_STEP_TIME
- v1.5 - fixed bug for esp boards
- v1.6 - Fixed stop for STEPPER4WIRE_HALF, speed can be set to float (for slow speeds)
- v1.7 - Fixed bug in negative speed (thanks to Evgeny Solodov)
- v1.8 - Fixed KEEP_SPEED mode
- v1.9 - Fixed bug with esp function max
- v1.10 - increased accuracy
- v1.11 - increased speed setting accuracy
- v1.12 - fixed smooth operation in KEEP_SPEED. Added support for "external" drivers. Removed SMOOTH argument from setSpeed
- v1.13 - minor bugs fixed, optimization
- v1.14 - fixed acceleration and deceleration bugs in KEEP_SPEED
- v1.15 - optimization, minor bugs fixed, stop() no longer resets maxSpeed
- v1.15.2 - added enabling EN if specified, even with autoPower disabled
- v2.0 - optimization. The core of the stepper is moved to a separate Stepper class. Added multi-axis trajectory planners
- v2.1 - added GyverStepper2, simplified and optimized version of GyverStepper
- v2.1.1 - fixed bug in GyverStepper
- v2.1.2 - Digispark compatibility
- v2.1.3 - fixed FOLLOW_POS in GStepper, fixed RELATIVE in GPlanner2 and fixed bug with jerks
- v2.1.4 - GPlanner2: fixed jerks, added adaptive rebuilding of the trajectory without stops, slightly optimized calculations
- v2.1.5 - ability to change speed and acceleration while the scheduler is running (GStepper2, GPlanner, GPlanner2)
- v2.1.6 - fixed compilation error when calling disable() in GStepper
- v2.1.7 - added clearBuffer() to GPlanner2
- v2.1.8 - optimization, fixed KEEP_SPEED in GStepper
- v2.2.0 - added speed profile GS_FAST_PROFILE for GStepper2, GPlanner, GPlanner2. Support for "tracking" mode for GStepper2
- v2.2.1 - small SRAM optimization
- v2.3 - fix compiler warnings, esp32 support
- v2.4 - improved smoothness of the movement of steppers in Planner and Planner2. Fixed bug in Stepper2
- v2.5 - fixed smooth speed change for KEEP_SPEED
- v2.6
    - disable() disables motor signal in virtual mode (for 4-wire drivers)
    - improved performance for step-dir drivers
    - added autoPower() to GStepper2
    - fixed jerk when changing direction in GStepper
- v2.6.1 - fixed bug in GStepper2
- v2.6.2 - optimized calculations in GStepper2, GPlanner and GPlanner2

<a id="feedback"></a>
## Bugs and feedback
When you find bugs, create an **Issue**, or better, immediately write to the mail [alex@alexgyver.ru](mailto:alex@alexgyver.ru)
The library is open for revision and your **Pull Request**'s!
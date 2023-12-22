This is an automatic translation, may be incorrect in some places. See sources and examples!

# Gyverstepper
Productive library for managing step engines with Arduino
- Support for 4 pine (step and hemisphere) and step-dir drivers
- automatic power outage when reaching the goal
- operating modes:
    - rotation with a given speed.Smooth acceleration and braking with acceleration
    - Following the position with the acceleration and limitation of speed
    - Following a position with a given speed (without acceleration)
- Fast algorithm for managing steps
- support of "virtual" drivers
- Built -in multi -axis trajectory planner

## compatibility
Compatible with all arduino platforms (used arduino functions)

### Documentation
There is [extended documentation] to the library (https://alexgyver.ru/gyverstepper/)

## Content
- [installation] (# Install)
- [STEPPERCORE] (# Core)
- [gyverstepper] (#stepper)
- [gyverstepper2] (# STEPPER2)
- [gyverplanner] (# Planner)
- [gyverplanner2] (# Planner2)
- [versions] (#varsions)
- [bugs and feedback] (#fedback)


## Ahhh why so many things?!
The library contains a set of tools for different scenarios for working with walking engines
-*STEPPERCORE.H*[Class ** STEPPER **]: the core of all other classes, knows how to quickly click with pins (AVR) and take one step for a configured type of driver.Supports 4 phases of the step/hemisphere, as well as STEP-dir drivers.
-*gyverstepper.h*[class ** gstepper **]: the main heavy library, a lot of settings.The movement of one motor with acceleration to a given position or rotation with a given speed.Not very optimal work in interrupting the timer.
-*gyverstepper2.h*[class ** gstepper2 **]: a new lightweight version of GyversStepper is almost completely compatible with it.A more optimal integer hybrid motion algorithm with acceleration, lightweight.Optimized to work in the interruption of the timer.
-*gyverplanner.h*[class ** gplanner **]: a multi -axis trajectory planner, acceleration (2 order).Stop at each point.The optimal work in interrupting the timer.
-*gyverplanner2.h*[class ** gplanner2 **]: a multi -axis trajectory planner, acceleration (2 order).Speed planning on the route, optimal traffic to points.The optimal work in interrupting the timer.

<a id="install"> </a>
## Installation
- The library can be found by the name ** gyverstepper ** and installed through the library manager in:
    - Arduino ide
    - Arduino ide v2
    - Platformio
- [download the library] (https://github.com/gyverlibs/gyversteper/archive/refs/heads/main.zip) .Zip archive for manual installation:
    - unpack and put in * C: \ Program Files (X86) \ Arduino \ Libraries * (Windows X64)
    - unpack and put in * C: \ Program Files \ Arduino \ Libraries * (Windows X32)
    - unpack and put in *documents/arduino/libraries/ *
    - (Arduino id) Automatic installation from. Zip: * sketch/connect the library/add .Zip library ... * and specify downloaded archive
- Read more detailed instructions for installing libraries [here] (https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%BD%D0%BE%BE%BE%BED0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)
### Update
- I recommend always updating the library: errors are corrected in new versions andCranberries, as well as optimization and new features are added
- through the IDE library manager: find the library how to install and click "update"
- Manually: ** remove the folder with the old version **, and then put a new one in its place.“Replacement” cannot be done: sometimes in new versions, files that remain when replacing are deleted and can lead to errors!


<a id="core"> </a>
## STEPPERCORE
### Description
The nucleus of the library for managing step engines:
- 4 phase and STEP DIR Drivers
- PIN support EN
- Virtual driver
- Quick IO algorithm for avr

<details>
<summary> Expand </summary>

### initialization
`` `CPP
STEPPER <TPPER2Wire> STEPPER (STEP, DIR);// Driver STEP-DIR
STEPPER <TPPER2Wire> STEPPER (STEP, DIR, EN);// Driver STEP-DIR + PIN Enable
STEPPER <stPper4Wire> stpeper (PIN1, PIN2, PIN3, PIN4);// driver 4 PIN
STEPPER <TPPER4Wire> stPper (PIN1, PIN2, PIN3, PIN4, EN);// Driver 4 PIN + Enable
STEPPER <STEPPER4Wire_HALF> STEPPER (PIN1, PIN2, PIN3, PIN4);// Driver 4 Pin Hole
STEPPER <STEPPER4Wire_HALF> STEPPER (PIN1, PIN2, PIN3, PIN4, EN);// Driver 4 Pin Hole + Enable

STEPPER <STEPPER2Wire, STEPPER_VIRTUAL> STEPPER;// STEP-DIR virtual driver
STEPPER <STEPPER4Wire, STEPPER_VIRTUAL> STEPPER;// virtual driver 4 pin
`` `

### Use (STEPPERCORE)
`` `CPP
VOID STEP ();// make a step
VOID Inverten (Bool Val);// Invert behavior en pina
VOID Reverse (Bool Val);// Invert the direction of the motor
Void Disable ();// Disconnect power and EN
VOID Enable ();// Turn on power and en
VOID Power (Bool);// Switch power
VOID Attachstep (VOID (*handler) (uint8_t));// Connect the step processor
VOID Attachpower (VOID (*handler) (bool));// Connect the food handler

int32_t pos;// current position in steps
int8_t dir;// Direction (1, -1)
`` `

### Example
The rest of the examples look at ** Examples **!
`` `CPP
#include <steppercore.h>
STEPPER <TPPER2Wire> STEPPER (2, 3);

VOID setup () {
  stepper.dir = 1;// or -1
  stepper.pos = 0;// access to position
}

VOID loop () {
  // Twist manually
  STEPPER.STEP ();// make a step
  Delay (10);
}
`` `
</details>

<a id="stepper"> </a>
## gyverstepper and gyverstepper2
- Gstepper2 is much easier and more optimized
- Gstepper2 is more designed to move along the trajectory points, but the "following" mode in it also works
- Gstepper2 is optimized to work in a timer interruption
- Gstepper2 uses a smooth motor motion algorithm by default
- Gstepper2 is likely to load the microcontroller a little more during operation than Gstepper.But much less than Gstepper in Smooth_algorithm mode

### How Gstepper works
- Fast algorithm (by default): Two timers were instituted in the library: a step timer (different period) and a planner timer (10-30 milliseconds).According to the planner, the trajectory is recalculated and the direction of movement and the current necessary speed is calculated.From this speed, a new period is obtained for the steps of steps.
- smooth algorithm (Smooth_algorithm settings): one step timer works, at every step the speed of movement and time is recalculated until the next step.

### How GSTEPPER2 works
- A smooth profile (by default): The Smooth_algorithm smooth motion algorithm is used here in Gstepper, but optimized and accelerated by 2-3 times.All heavy calculations of the trajectories are made when setting a new target position in Settarget.Further along the steps of steps, the motor steps and quickly calculates the new time of the next step.This allows you to spend a minimum of time interrupting the timer if it is used.
- Fast profile (Setting GS_FAST_PROFILE): Everything is the same, but in the steps of steps there is no calculation, but the choice of the period from the pre -calculated table, which allowsis to be accelerated up to speeds of 30,000 steps per second.

## gyverstepper
### Description
Library for managing step engines with Arduino
- Support for 4 pine (step and hemisphere) and step-dir drivers
- automatic power outage when reaching the goal
- operating modes:
    - rotation with a given speed.Smooth acceleration and braking with acceleration
    - Following the position with the acceleration and limitation of speed
    - Following a position with a given speed (without acceleration)

<details>
<summary> Expand </summary>

## H initialization
`` `CPP
// stps - steps for one turn of the shaft (for calculations with degrees)
// STEP, DIR, PIN1, PIN2, PIN3, PIN4 - Any GPIO
// en - Pin disconnecting the driver, any GPIO
Gstepper <stepper2wire> stepper (Steps, STEP, DIR);// Driver STEP-DIR
Gstepper <stepper2wire> stepper (Steps, STEP, DIR, EN);// Driver STEP-DIR + PIN Enable
GSTEPPER <TPPER4Wire> STEPPER (STEPS, PIN1, PIN2, PIN3, PIN4);// driver 4 PIN
GSTEPPER <TPPER4Wire> STEPPER (STEPS, PIN1, PIN2, PIN3, PIN4, EN);// Driver 4 PIN + Enable
Gstepper <stPper4wire_HALF> stPper (STEPS, PIN1, PIN2, PIN3, PIN4);// Driver 4 Pin Hole
Gstepper <stepper4wire_half> stepper (stps, Pin1, PIN2, PIN3, PIN4, EN);// Driver 4 Pin Hole + Enable
GSTEPPER <stPper4Wire, STEPPER_VIRTUAL> STEPPER (2048);// virtual driver, indicate only the number of steps
`` `

### Usage
`` `CPP
// Note: Further in the text by default, I mean "even if not to cause a function"

// The movement of the motor occurs here, call as often as possible!
// has a built -in timer
// Returns True if the motor moves to the target or spins along Keep_Speed
Bool Tick ();

// Returns the same as Tick, i.e.The motor is spinning or not
Bool getstate ();

// Invert the direction of the motor - True (by default. False)
VOID Reverse (Bool Dir);

// Invert the behavior en Pina - True (by silence. False)
VOID Inverten (Bool Rev);

// Installation of the operating mode, mode:
// follow_pos - following the position of Settarget (...)
// keep_Speed - holding speed of setspeed (...)
VOID Setrunmode (GS_RUNMODE MODE);

// Installation of the current position of the motor in steps and degrees
VOID setcurrent (LONG POS);
VOID setcurrentdeg (Float POS);

// Reading the current position of the motor in steps and degrees
Long getcurrent ();
Float getcurrentdeg ();

// Installation of the target position in steps and degrees (for the FOLLOW_POS mode)
// type - absolute or relative, by default costs absolute
VOID settarget (LONG POS);
VOID settarget (LONG POS, GS_Postype Type);
Void settargetdeg (Float POS);
VOID settargetdeg (Float Pos, GS_Postype Type);

// Obtaining the target position in steps and degrees
Long gettarget ();
Float gettargetdeg ();

// Installation of maximum speed (according to the module) in steps/second and degrees/second (for Follow_POS mode)
// by the silence.300
VOID setmaxSpeed (Float Speed);
VOID setmaxSpeeddeg (Float Speed);

// Installation of acceleration in steps and degrees per second (for the Follow_POS mode).
// at a value of 0 acceleration is turned off and the motor works
// according to the profile of constant maximum speed setmaxSpeed ().
// by the silence.300
VOID setaccoleration (Intscel);
VOID setaccolerationdeg (Float Accel);

// Auto Detection EN when reaching the position - True (by the silence. False)
Void Autopower (Bool Mode);

// smooth stop with a given acceleration
VOID Stop ();

// Hard stop
VOID Brake ();

// hard stop + reset of position in 0 (for endings)
VOID Reset ();

// Installation of targeted speed in steps/second and degrees/second (for Keep_Speed mode)
VOID SetSpeed (Float Speed);
VOID SetSpeeddeg (Float Speed);

// Obtaining targeted speed in steps/second and degrees/second (for Keep_Speed mode)
Float GetSpeed ();
Float getspeeddeg ();

// Turn on the motor (PIN EN)
VOID Enable ();

// Turn off the motor (PIN EN)
Void Disable ();

// Returns the minimum period of the tick of the motor in microseconds at the settled setmaxSpeed () speed.
// can be used to configure the timer interruptions, in the processor of whichwow will be tick () (see Timerisr example)
uint16_t getminperiod ();

// The current period "TIKA" for debugging and all of this
uint16_t steptime;

// connect an external processor for step and switch power
Void Attachstep (Handler)
Void Attachpower (Handler)
`` `

### Example
The rest of the examples look at ** Examples **!
`` `CPP
#include <gyverstepper.h>
GSTEPPER <TPPER4Wire> STEPPER (2048, 5, 3, 4, 2);

VOID setup () {
  Serial.Begin (115200);
  // Speed maintenance mode
  STEPPER.SETRUNMODE (keep_Speed);

  // you can set the speed
  STEPPER.SETSPEED (120);// in steps/second
  STEPPER.SETSPEEDDEG (80);// in degrees/s

  // Message to the target position
  STEPPER.SETRUNMODE (follow_pos);

  // you can establish a position
  STEPPER.SETTARGET (-2024);// in steps
  STEPPER.SETTARGETDEG (-360);// In degrees

  // Installation Max.speeds in degrees/s
  STEPPER.SetmaxSpeeddeg (400);

  // Installation Max.Speed in steps/second
  STEPPER.SetmaxSpeed (400);

  // Installation of acceleration in degrees/second
  STEPPER.SETACCELERATIONDEG (300);

  // Installation of acceleration in steps/sec/s
  STEPPER.SETACCELERATION (300);

  // Disconnect the motor when reaching the goal
  STEPPER.Autopower (True);

  // Turn on the motor (if the pin is specified)
  STEPPER.ENABLE ();
}

VOID loop () {
  // just twist tudes-fuckers
  if (! stepper.tick ()) {
    Static Bool Dir;
    Dir =! Dir;
    STEPPER.SETTARGET (DIR? -1024: 1024);
  }
}
`` `
</details>

<a Id="stepper2"> </a>
## gyverstepper2
### Description
Lightened GyverstePper
- easier for a few KB, all the integer
- increased accuracy of positioning
- more effective hybrid motion algorithm
- Movement to the goal with acceleration
- rotation at speed
- Max.speed:
  - Conventional mode: 37,000 step/s on full, 18,000 step/s on acceleration
  - Fast profile: 37000 step/s on full, 37,000 step/s on acceleration
- optimized for the interruption of the timer
- Inherits STEPPERCORE

### Logic of Work
- settarget ()/settargetdeg () sends the motor to the specified position
- The movement occurs in Tick (), which needs to be interviewed constantly.Either in Tickmanual, which must be called with a period received from Getperiod () and recount on each step
- Tick () will return True if the motor is spinning
- Ready () will return True once if the motor has reached the target and stopped
- while moving towards the target, you can call PAUSE (), then the engine will reach the point and stop, Ready () will not return True
- while moving towards the target, you can cause Stop (), then the motor will slow down with a given acceleration, Ready () will not return True
- while moving towards the target, you can call Brake (), then the motor will stop, Ready () will not return True
- after stopping, you can call resume (), the motor will continue to move to the target
- Constant rotation is set by SetSpeed ()/setSpeeddeg ().You can stop abruptly - Stop () or Brake ()
- Speed and acceleration can be set at any time, but they are used after stopping the motor!

<details>
<summary> Expand </summary>

## H initialization
`` `CPP
GSTEPPER2 <TPPER2Wire> STEPPER (WHECHOOL REGULATION, STEP, DIR);// Driver STEP-DIR
GSTEPPER2 <TPPER2Wire> STEPPER (WHECHOOL REGULATION, STEP, DIR, EN);// Driver STEP-DIR + PIN Enable
GSTEPPER2 <TPPER4Wire> STEPPER (WHECHOOL, PIN1, PIN2, PIN3, PIN4);// driver 4 PIN
GSTEPPER2 <TPPER4Wire> STEPPER (WHECHOOL, PIN1, PIN2, PIN3, PIN4, EN);// Driver 4 PIN + Enable
Gstepper2 <stPper4Wire_HALF> stPper (step -off, PIN1, PIN2, PIN3, PIN4);// Driver 4 Pin Hole
GSTEPPER2 <stPper4Wire_HALF> STEPPER (Shagyna Two, PIN1, PIN2, PIN3, PIN4, EN);// Driver 4 Pin Hole + Enable

GSTEPPER2 <stPper2Wire, STEPPER_VIRTUAL> STEPPER;// STEP-DIR virtual driver
Gstepper2 <stPper4wire, stpeper_virtual> stPper;// virtual driver 4 pin
`` `

### Usage
`` `CPP
// === Inherited from stepper =====
VOID STEP ();// make a step
VOID Inverten (Bool Val);// Invert behavior en pina
VOID ReVerse (Bool Val);// Invert the direction of the motor
Void Disable ();// Disconnect power and EN
VOID Enable ();// Turn on power and en
VOID Attachstep (VOID (*handler) (uint8_t));// Connect the step processor
VOID Attachpower (VOID (*handler) (bool));// Connect the food handler

int32_t pos;// current position in steps
int8_t dir;// Direction (1, -1)

// ========= gstepper2 =============
// ticker
Bool Tick ();// Move ticker, call often.Will return True if the motor moves
Bool Tickmanual ();// Hand Ticker for calling in interrupting the timer with the Getperiod () period.Will return True if the motor moves
Bool Ready ();// True will return once if the motor has reached the established position and stopped

// rotation
VOID SetSpeed (Int16_T Speed);// set speed in steps/s and start rotation
VOID SetSpeed (Float Speed);// set speed in steps/seconds and start rotation

// Movement to the goal
VOID settarget (int32_t ntar, gs_postype type = Absolute);// Set the target in steps and optionally Absolute/Relative mode
VOID settargetdeg (int32_t ntar, gs_postype type = absolute);// set a target in degrees and optionally Absolute/Relative mode
int32_t gettarget ();// Get a target position in steps

VOID setaccoleration (uint16_t na);// Installation of acceleration in the step/sec^2
VOID SetmaxSpeed (Intsed);// set the speed of movement when following the settarget () position in steps/s
VOID setmaxSpeed (Float Speed);// set the speed of movement when following the settarget () position in steps/sec, float
VOID setmaxSpeedDeg (int spEED);// set the speed of movement when following a position in the city/s
VOID setmaxSpeeddeg (Float Speed);// Set the speed of movement when following a position in hail/s, float

VOID setcurrent (int32_t npos);// set the current position
int32_t getcurrent ();// get the current position
VOID Reset ();// drop the current position in 0

// Everyone
Void Autopower (Bool Mode);// Auto Detachment of the motor when reaching the position - True (by silence. False)
uint32_t getperiod ();// get the current period of ticks
VOID Brake ();// stop the motor abruptly
VOID PAUSE ();// pause - get to a given point and wait (Ready () will not return True until you are on a pause)
VOID Resume ();// Continue movement after stopping/pause
uint8_t getstatus ();// current status: 0 - stand, 1 - go, 2 - we go to the point of the pause, 3 - we spin at speed, 4 - inhibit

// ===== defines settings ========
// definate before connecting the library
#define GS_NO_ACCEL // Disable the movement module with acceleration (reduce code weight)

#define gs_fast_profile size_massive (for example 10)
Includes a fast speed planner.Acceleration/braking area is broken
For the indicated number of segments (+8 bytes of SRAM to the site), the speed will be the same on them.
This allows you to quickly calculate the speed of the motor and reach 30,000 step/s on the acceleration site
(in normal mode, half as much).
`` `

### Example
The rest of the examples look at ** Examples **!
`` `CPP
// Twist here, tick in look

#include "gyverstepper2.h"
GSTEPPER2 <TPper2Wire> STEPPER (2048, 2, 3);

VOID setup () {
  Serial.Begin (9600);
  //stepper.enable ();
  STEPPER.SetmaxSpeed (100);// speed to the target
  STEPPER.SETACCELERATION (200);// Acceleration
  STEPPER.SETTARGET (300);// target
}

Bool Dir = 1;
VOID loop () {
  STEPPER.Tick ();// Motor asynchronously spins here

  // if you arrive
  if (stepper.read ()) {
    Dir =! Dir;// unfold
    STEPPER.SETTARGET (DIR * 300);// We go the other way
  }

  // asynchronous conclusion to the port
  Static uint32_t tmr;
  if (millis () - tmr> = 30) {
    TMR = Millis ();
    Serial.println (stpeper.pos);
  }
}
`` `
</details>


<a id="planner"> </a>
## gyverplanner
### Description
A multi -axis planner of trajectories for walking engines and creating a CNC machine
- Stop at each point.One next position
- Max.speed:
  - Conventional mode: 37,000 step/s on full, 14,000 step/s on acceleration
  - Fast profile: 37000 step/s on full, 37,000 step/s on acceleration
- trapezoidal speed profile (2nd order planner)
- setting speed and acceleration
- Any number of axes.Will move synchronously to the set goals
- fast integer model of path planning and speed
- regime of constant rotation for one axis (for movement towards the end for example)
- Brake/smooth stop/pause on the planner trajectory
- optimized for the interruption of the timer
- Fast control of steps steps for Arduino avr
- Speed and acceleration can be set at any time, but they are used after stopping the motor!

### Logic of Work
The planner controls any number of engines, rotating them to the indicated position.In this version
stops at each point of the trajectory, after which the flag of Ready () rises and expects
installations of the next point.
- See the simulation in Processing: Planner Simulation/Planner folder

<details>
<summary> Expand </summary>

## H initialization
`` `CPP
GPlanner <driver, number of axes> Planner;
`` `

### Usage
`` `CPP
VOID Addstepper (Uint8_T Axis, STEPPER & STP);// Connect the STEPPER class motor on the axis axis
// Note: the type of driver should match the planner and motors

VOID Setbacklash (Uint8_t Axis, Uint16_T Steps);// Set the back of the ramp on the axis axis in the number of steps steps
VOID Enable ();// Turn on the motors
Void Disable ();// Turn off the motors
VOID Power (Bool V);// Switch power

// SETTINGS
VOID setmaxSpeed (Float NV);// Installation of the maximum speed of the planner in the step/s
VOID setaccoleration (uint16_t na);// Installation of acceleration of the planner in the step/sec^2

// planner
uint32_t getperiod ();// returns time to the ISS until the next call Tick/Tickmanual
Bool Ready ();// True - ready to accept the next point of the route
VOID PAUSE ();// pause (get to a given point and wait).Ready () will not return True until you are on a pause
VOID Stop ();// Stop smoothly (with a given acceleration)
VOID Brake ();// sharply stop the motors from any regime
VOID Resume ();// Continue after stopping/pauses
VOID Reset ();// throw off the counters of all motors in 0
VOID Home ();// Send to 0 for all axes
uint8_t getstatus ();// current status: 0 - stand, 1 - go, 2 - we go to the point of the pause, 3 -t - we circle at speed

// SPEED
VOID SetSpeed (Uint8_t Axis, Float Speed);// Permanent rotation mode for axis axis at the speed of Speed Step/S (MB Neutheity)

// Position
VOID setcurrent (int16_t cur []);// establish the current position of the engines
VOID setcurrent (int32_t cur []);// establish the current position of the engines
Int32_T Getcurrent (int Axis);// Get the current position on the axis axis

// set the target in steps and start movement.Type - Absolute
// Absolute - specific coordinates of the point where to move
// Relative - displacement relative to the current motors provisions
// will return True if the goal is set.FALSE if the goal coincides with the current
Bool settarget (int32_t target []);
Bool settarget (int16_t target []);
Bool settarget (int32_t target [], type);
Bool settarget (int16_t target [], type);
Int32_T Gettarget (int Axis);// Get a target in steps on axis axis

// ticker
// ticker, call as often as possible.Will return True if the motor is spinning
// Steps are taken here both for movement by points and for speed rotation
Bool Tick ();

// Hand ticker for a call in an interruption or somewhere else.20..50 US is performed
Bool Tickmanual ();

// ======= The defines of settings ==========
// announce before connecting the library
#define gs_fast_profile size_massive (for example 10)
Includes a fast speed planner.Acceleration/braking area is broken
For the indicated number of segments (+8 bytes of SRAM to the site), the speed will be the same on them.
This allows you to quickly calculate the speed of the motor and reach 30,000 step/s on the acceleration site
(in normal mode, half as much).
`` `

### Example
The rest of the examples look at ** Examples **!
`` `CPP
// Basic example: how to create and launch a planner
// When starting, the motors will be sent to the first position
// when reaching - on the second.After that, the movement will stop
// Open Plotter and see graphs

#incLude "gyverplanner.h"
// Create the motors of the STEPPER class indicating the type of driver and pins
// Motors should be with the same type of driver
// Here they are handsome
STEPPER <TPPER2Wire> STEPPER1 (2, 3);
STEPPER <TPPER2Wire> STEPPER2 (4, 5);

// Create a planner, indicate in <> the type of driver like motors
// and the number of axes equal to the number of engines (any more than 1)
GPlanner <stPper2Wire, 2> Planner;

VOID setup () {
  Serial.Begin (115200);
  // Add steps to the axis
  Planner.Addstepper (0, STEPPER1);// axis 0
  Planner.Addstepper (1, STEPPER2);// axis 1

  // set acceleration and speed
  Planner.Setacceleration (100);
  Planner.SetmaxSpeed (300);

  Planner.Reset ();// Reset all positions in 0 (they are already in 0 at launch)

  // array with target positions of the axes, the size of the array is equal to the number of axes
  int target [] = {300, 200};

  // Send
  Planner.Settarget (Target);
}

VOID loop () {
  // here is the movement of motors, call as often as possible
  Planner.Tick ();

  // will return True if all the engines have arrived
  if (Planner.read ()) {
    // Download a new point
    int Newtarget [] = {10, 50};
    Planner.Settarget (Newtarget);
  }

  // asynchronously I bring graphics to the port
  Static uint32_t tmr;
  if (millis () - tmr> = 20) {
    TMR = Millis ();
    Serial.print (stPper1.pos);
    Serial.print (',');
    Serial.println (stPper2.pos);
  }
}
`` `
</details>

<a id="planner2"> </a>
## gyverplanner2
### Description
A multi -axis planner of trajectories for walking engines and creating a CNC machine
- Speed planning on the route.Customer buffer
- Max.speed:
  - Conventional mode: 37,000 step/s on full, 14,000 step/s on acceleration
  - Fast profile: 37000 step/s on full, 37,000 step/s on acceleration
- trapezoidal speed profile (2nd order planner)
- setting speed and acceleration
- Any number of axes.Will move synchronously to the set goals
- fast integer model of path planning and speed
- regime of constant rotation for one axis (for movement towards the end for example)
- Brake/smooth stop/pause on the planner trajectory
- optimized for the interruption of the timer
- Fast control of steps steps for Arduino avr
- Speed and acceleration can be set at any time, but they are used after stopping the motor!

### Logic of Work
The planner controls any number of engines, rotating them to the indicated position.In this version
A trajectory buffer is implemented, which can be filled with dots while Available () returns True.
Addtarget () accepts:
- array of points specified in the initialization of the size
- Stop flag.If you transfer 1 - the planner stops the engine at this point and will wait for the further resume () command ()
- Type of point: absolute (absolute coordinate) or relative (relative to the previous point)

Когда плаанировщик приезжает до точки остановки - он встаёт на паузу (например для включения выключенияtool),
After performing the necessary actions, we call Resume () and he continues to move.
Unlike the previous GPlanner, the GPlanner2 has been implemented by the path of the path in the buffer and planning
speeds for all points, which allows the system to move faster and not slow down at each point.
- See the simulation in Processing: Planner Simulation/Planner2 folder

<details>
<summary> Expand </summary>

## H initialization
`` `CPP
GPlanner2 <driver, number of axes> Planner;// Announcement
GPlanner2 <driver, number of axes, buffer size> Planner;// + buffer size (in silence 32)
`` `

### Usage
`` `CPP
VOID Addstepper (Uint8_T Axis, STEPPER & STP);// Connect the STEPPER class motor on the axis axis
// Note: the type of driver should match the planner and motors

VOID Setbacklash (Uint8_t Axis, Uint16_T Steps);// Set the back of the ramp on the axis axis in the number of steps steps
VOID Enable ();// Turn on the motors
Void Disable ();// Turn off the motors
VOID Power (Bool V);// Switch power

// SETTINGS
VOID setmaxSpeed (Float NV);// Installation of the maximum speed of the planner in the step/s
VOID setaccoleration (uint16_t na);// Installation of acceleration of the planner in the step/sec^2
VOID Setdta (Float Newdta);// Install DT shift shifts in a turn, 0.0 .. 1.0.0.3

// planner
uint32_t getperiod ();// returns time to the ISS until the next call Tick/Tickmanual
VOID Start ();// Start work
VOID Stop ();// Stop smoothly (with a given acceleration)
VOID Brake ();// sharply stop the motors from any regime
VOID Resume ();// Continue after stopping or the final point of the route
VOID Reset ();// throw off the counters of all motors in 0
Bool Ready ();// Flag of reaching a stop point.After it you need to call resume
Bool Available ();// True - there is a place for a new point in the planner's buffer

uint8_t getstatus ();// current status:
// 0 Waiting for the command (stopped)
// 1 waiting for the buffer
// 2 on the way
// 3 per pause
// 4 on stop
// 5 SetSpeed is spinning

// SPEED
VOID SetSpeed (Uint8_t Axis, Float Speed);// Permanent rotation mode for axis axis at the speed of Speed Step/S (MB Neutheity)

// Position
// Add a new point of the route.Coordinate array, ending flag and absolute/relative
VOID Addtarget (Int32_t Tar [], Uint8_t L, GS_Postype Type = Absolute);
VOID Addtarget (int16_t tar [], uint8_t l, gs_postype test = absolute);
// Absolute - specific coordinates of the point where to move
// Relative - displacement relative to the current motors provisions

VOID setcurrent (int16_t cur []);// establish the current position of the engines
VOID setcurrent (int32_t cur []);// establish the current position of the engines
Int32_T Getcurrent (int Axis);// Get the current position on the axis axis
Int32_T Gettarget (int Axis);// Get the current target in steps on the axis axis

// ticker
// ticker, call as often as possible.Will return True if the motor is spinning
// Steps are taken here for movement by points, for rotation in speed, as well as restructuring the buffer
Bool Tick ();

// Hand ticker for a call in an interruption or somewhere else.20..50 US is performed
Bool Tickmanual ();

// Buffer handler.It itself is called in Tick.It is necessary to call manually when working with Tickmanual
// will return True if the planner sent motors to a new position (at that moment you can run the timer)
Void Checkbuffer ();

// ======= The defines of settings ==========
// announce before connecting the library
#define gs_fast_profile size_massive (for example 10)
Includes a fast speed planner.Acceleration/braking area is broken
for the indicated number of segments (+8 bytes SRAM for the site), onThe cranberries will be the same.
This allows you to quickly calculate the speed of the motor and reach 30,000 step/s on the acceleration site
(in normal mode, half as much).
`` `

### Example
The rest of the examples look at ** Examples **!
`` `CPP
// Example with a route recorded in memory
// See the schedule, or better start the STEPPERPLOT

int path [] [2] = {{
  {100, 250},
  {160, 30},
  {230, 250},
  {60, 100},
  {270, 100},
};

// number of points (let the compiler consider it)
// as the weight of the total array / (2+2) byte
int nodeamount = sizeof (Path) / 4;

#include "gyverplanner2.h"
STEPPER <TPPER2Wire> STEPPER1 (2, 3);
STEPPER <TPPER2Wire> STEPPER2 (4, 5);
GPlanner2 <STEPPER2Wire, 2> Planner;

VOID setup () {
  Serial.Begin (115200);
  // Add steps to the axis
  Planner.Addstepper (0, STEPPER1);// axis 0
  Planner.Addstepper (1, STEPPER2);// axis 1

  // set acceleration and speed
  Planner.Setacceleration (500);
  Planner.SetmaxSpeed (500);

  // The starting point of the system should coincide with the first point of the route
  Planner.Setcurrent (Path [0]);
  Planner.Start ();
}

int// Mainplane Mixture meter
VOID loop () {
  // here is the movement of motors, call as often as possible
  Planner.Tick ();

  // If there is a place in the planner's buffer
  if (Planner.available ()) {
    // Add the point of the route and whether it is a stop point (0 - no)
    Planner.Addtarget (Path [Count], 0);
    IF (++ Count> = Sizeof (Path) / 4) Count = 0;// Clothing
  }

  // asynchronously I bring graphics to the port
  Static uint32_t tmr;
  if (millis () - tmr> = 20) {
    TMR = Millis ();
    Serial.print (stPper1.pos);
    Serial.print (',');
    Serial.println (stPper2.pos);
  }
}
`` `
</details>

<a id="versions"> </a>
## versions
- V1.1 - added the possibility of smooth speed control in Keep_Speed (see example of acceldeccelbutton)
- V1.2 - Added support ESP8266
- V1.3 - Changed the logic of Settarget (, Relative)
- v1.4 - added a delay for STEP, you can configure the define Driver_Step_Time
- V1.5 - Filled Bag for Plates ESP
- V1.6 - Fixed a stop for STEPPER4Wire_HALF, speed can be set in Float (for slow speeds)
- V1.7 - Bug corrected in negative speed (thanks to Evgeny Solodov)
- V1.8 - Fixed Keep_Speed mode
- V1.9 - Fixed error with ESP function max
- V1.10 - increased accuracy
- v1.11 - increased accuracy of the speed task
- V1.12 - The smooth work in Keep_Speed is featured.Added support for "external" drivers.The SMOOTH argument from SetSpeed is removed
- v1.13 - small bugs, optimization are fixed
- V1.14 - Fixed errors of dispersal and braking in Keep_Speed
- V1.15 - Optimization, small bugs are fixed, Stop () no longer drops MaxSpeed
- v1.15.2 - added enable EN if specified, even with Autopower disconnected
- V2.0 - optimization.The nucleus of the step is taken into a separate STEPPER class.Added multi -axial trajectory planners
- V2.1 - added Gyverstepper2, a simplified and optimized version of Gyverstepper
- V2.1.1 - Fixed a bug in Gyverstepper
- V2.1.2 - Digispark compatibility
- V2.1.3 - Follow_POS repaired in Gstepper, repaired Relative in GPlanner2 and corrected the bug with jerks
- V2.1.4 - GPlanner2: corrected jerks, added adaptive rebuilding the trajectory without stops, a bit optimized the calculations
- V2.1.5 - the ability to change speed and acceleration during the planner’s operation (GSTEPPER2, GPlanner, GPlanner2)
- V2.1.6 - Fixed compilation error when calling Disable () to Gstepper
- v2.1.7 - added Clearbuffer () to gplanner2
- V2.1.8 - Optimization, fixed Keep_Speed to gstepper
- V2.2.0 - added the high -speed GS_FAST_PROFILE profile for gstepper2, GPlanner, GPlanner2.Supporting the "tracking" mode for gstepper2
- V2.2.1 - Small SRAM optimization
- V2.3 - FIX Compiler Warnings, ESP32 support
- V2.4 - increased smoothness of the movement of steps in Planner and Planner2.Fixed a bug in STEPPER2
- v2.5 - Fixed a smooth speed change for Keep_Speed
- V2.6
    - Disable () in virtual mode disconnects the signal from the motor (for 4-provdrivers)
    - Improved performance for STEP-DIR drivers
    - added Autopower () to gstepper2
    - Fixed a jerk when changing the direction to Gstepper
- V2.6.1 - Corrected Baga in Gstepper2
- V2.6.2 - Optimized calculations in Gstepper2, GPlanner and GPlanner2
- V2.6.3 - Reverse () in the STEP -dir driver is now used immediately
- V2.6.4 - Fixed Relative Settarget () at Gplanner https://github.com/gyverlibs/gyverstepper/pull/11
- V2.7
  - Fixed various compilation errors
  - Fixed some critical bugs
  - Added power management to the planners
  - added backlash compensation to planners
  - Fixed bugs with zero acceleration in all libraries
  - Fixed a slow stop and a blow with a large acceleration
  - increased performance for ESP8266

<a id="feedback"> </a>
## bugs and feedback
Create ** Issue ** when you find the bugs, and better immediately write to the mail [alex@alexgyver.ru] (mailto: alex@alexgyver.ru)
The library is open for refinement and your ** pull Request ** 'ow!


When reporting about bugs or incorrect work of the library, it is necessary to indicate:
- The version of the library
- What is MK used
- SDK version (for ESP)
- version of Arduino ide
- whether the built -in examples work correctly, in which the functions and designs are used, leading to a bug in your code
- what code has been loaded, what work was expected from it and how it works in reality
- Ideally, attach the minimum code in which the bug is observed.Not a canvas of a thousand lines, but a minimum code
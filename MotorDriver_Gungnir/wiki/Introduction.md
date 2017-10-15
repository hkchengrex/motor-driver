# Introduction
Gungnir is a generic motor driver program, with PID constants tuned for Faulhaber 3863 24V. It uses encoder feedback and controls PWM and DIR output as to control the velocity and position of the motor. Communication protocol is in CAN. Made for motor controller 3.2 and 5.2 (better tuned for 5.2). Program and wiki written by Rex Cheng, for Robocon 2017.

> “Óðinn á yðr alla!” (“Odin owns all of you!”)

## Features
- Open loop [control](command.md) with certain PWM
- Close loop [control](command.md) on velocity and position
	- Travel to target position (with trapezoidal [path](path.md))
	- Maintain velocity and position traveled (integrated velocity)
- Max. velocity adjustment
- Acceleration adjustment
- [Encoder and PWM feedback](feedback.md)
- [Path Feedback](path.md)
- [Debug mode](debug_mode.md) for hardware
- [UART feedback](feedback.md) for debugging
- [LED](led.md) life signal
- [Feedback and auto-recovery in case of encoder failure](encoder_fail.md)
- [Competition Mode](competition_mode.md)
- [AT Mode](at_mode.md)
- [Limit switches](limit.md)

* Before sending any commands to the motor driver, please allow some time for the motor driver to boot (like delay for 100ms), especially for value-setting commands like acceleration and max. velocity*

## Known issue
1. During deceleration phase (especially significant when the main board also perform deceleration, i.e. velocity change commands are sent continuously), the motor's direction pin will flash (which is normal), and the motor will produce "tak tak" sound (which is abnormal). Usually, it is because the motor do need to change direction pin (as it needs force other than friction to decelerate quick enough), but the PWM-Force curve is non-continuous around zero (aka hardware braking). Usually the issue can be eased by giving it some load (putting one's hand on it is usually enough). Sometimes it can also occur during constant speed phase and I have no idea why.

2. Position uses 32 bit. It will overflow. Guaranteed not to happen in a 3-minute game though. Using 64 bit will make it buggy, and I have no idea (again) why. Note: Tried to use 64 bits for weeks, no luck. Maybe because it is not atomic?

## Constant
Property | Value | Reference
--- | --- | ---
Maximum PWM | 7999 | 
Control frequency | 512 Hz | 
Default acceleration | 900000 | 
Default maximum velocity | 150000 | 
Encoder feedback frequency | Every 2 ms | 

## Units
- Position in encoder count
- Velocity in encoder count per second
- Acceleration in encoder count per second squared
- For our encoder system with ~14 gear ratio (measured to be around 13.79?), each rotation should give 512(encoder resolution)*4(Multiplier)*13.79(Gear ratio).

> "I take it from this that you are an Accelerationist?"  
> "No, " said Sam, "simply an inquirer. I am curious, that's all, as to the reasons."  
> ~Lord of Light  

Inspired by Micheal Siu and Taraka, a powerful Rakasha.

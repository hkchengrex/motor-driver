# Limit switches

Finally we use limit switches to actually limit actions...

## Installation
Set ACTIVATE_HIGH_LIMIT_SWITCH / ACTIVATE_LOW_LIMIT_SWITCH in define.h. The high limit switch refers to the limit switch at the direction of positive encoder ticks and positive PWM. Note that the switches in the 5.2 motor driver have hardware pull-up.

## Limiting property
The limit switch has a very high priority. It strictly prohibit any PWM that tries to go further into the triggered limit switch to actuate. For instance, when the high limit switch is triggered, any positive PWM will be converted to zero at a low level, overriding PID and user-set PWM value.

The motor driver will save an internal encoder ticks at the boundary of the limit switch activation range. It is defined to be the encoder ticks at the moment when the limit switch is triggered. When the limit switch is triggered and a new path that is pointing at the prohibited direction is generated, that path will be invalidated, and another new path will be generated towards the boundary encoder ticks in close loop. This will also happens when the limit switch changes from not triggered to triggered. Note that this path would still have acceleration. In open loop, a zero pwm will be applied.

## Feedback
Whenever the boundary encoder ticks is updated, the motor driver will issue a feedback signal to the mainboard, including the boundary encoder ticks.

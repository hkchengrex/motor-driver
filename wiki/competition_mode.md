# Competition Mode

Competition mode can be enabled by sending the enable flag to the motor driver.

Normally, after a encoder malfunction is detected, the motor driver would give zero pwm and enter recovery routine (see [encoder failure](encoder_fail.md)). In competition mode, we would like to ensure the robot still "kinds of working". A mini controller will be enabled to do the following things:

- Map close loop velocity to open loop velocity. This would simply convert the percentage close loop velocity (out of the max. velocity as set by the user or as default) to percentage PWM.
- Acceleration will be applied. It is hardcoded to be (512/7999) duty cycle per control cycle.
- Hardware braking will be applied. A zero pwm in the input would be converted to pwm of 1 in the opposite direction with the original direction to effectively brake the motor without any kind of control. (For instance, if pwm changes from 3000->1000->0, it would be converted to 3000->1000->-1).

This would not be activated in the case of position control.

Once encoder signal is caught, everything would resume normal.
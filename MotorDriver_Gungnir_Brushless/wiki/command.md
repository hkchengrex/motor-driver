# Underlying Protocol
See [CAN motor protocol](protocol.md). Note that the board need some time to boot up. Main board should put some delay instead of sending command immediately on start.

## Open loop control
Control the PWM directly. Range: -7999~7999.

## Close loop position control
Control the position by generating a trapezoidal path using the maximum velocity and acceleration at the received instant. Range: 32 bit integer.

## Close loop velocity control
Control the velocity and position traveled (which is integrated from velocity). This will generate half of the trapezoidal path. Range: -MAX_VEL~MAX_VEl.

## Maximum velocity adjustment
Adjust the maximum velocity used in path generation. Default: 150000. Unit: Count per second.

## Acceleration adjustment
Adjust the acceleration (slope) used in path generation. Default: 900000. Unit: Count per second squared.

## Motor lock
Immediately set the target position and target velocity to zero without applying acceleration.

## [Competition Mode](competition_mode.md)

## Silence Mode
Shut up the motor by doubling the motor frequency.
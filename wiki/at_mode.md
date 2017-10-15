# AT Mode
You need not to short any pin to use AT mode, as there is no need to enter/leave AT mode. 

## Specification
- UART1
- Baud: 115200
- Stop Bit: 1
- Parity Bit: None
- End of line: CR (\r)

## Commands
Command | Respond | Note
--- | --- | ---
AT | OK | 
AT+SHUT | OK | Shut down the UART feedback
AT+LIVE | OK | Bring UART feedback back to life
AT+BUILD? | The compile date and time | 
AT+MOTOR_ID? | Motor id for this motor | 
AT+MOTOR_ID=<Param> | OK | Note: *Only temporary, not saved in flash* Param can be 0~15
AT+CONTROL_FREQ? | Control frequency | Read only
AT+LIST_VALUE | Current position, velocity, PWM, acceleration, max. velocity | 
AT+ACC=<Param> | OK | Default: 900000
AT+MAX_VEL=<Param> | OK | Default: 150000
AT+PATH_POS=<Param> | Information of the path generated | This will actually move the motor
AT+PATH_VEL=<Param> | Information of the path generated | This will actually move the motor
AT+PWM=<Param> | OK | View the pwm with AT+LIST_VALUE
AT+DEBUG | OK | The device will enter debug mode
AT+LIMIT? | Show the limit switch activation (set during compilation) and current input | 

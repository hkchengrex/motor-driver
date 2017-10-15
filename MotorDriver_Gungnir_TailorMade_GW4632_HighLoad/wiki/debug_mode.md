# Debug mode

## Entering Debug Mode
There are two ways to enter debug mode.
1. By flashing a program compiled with DEBUG_MODE defined. The program will be forced to enter debug mode every time it runs.
2. By using a normal program, but with special jumper connected. For motor controller 3.2, connect the middle two pins of UART3. For motor controller 5.2, connect the rightmost two pins of SWD. If the board is powered up with such configuration, it will enter debug mode.

## Debug mode routine
1. Wake up and yawn for 1 second.
2. Move in close loop positive direction for 3 seconds.
3. Lock in position for 3 seconds.
4. Target encoder zero position (original position) until it reaches target.
5. Sleep again with position lock kept. Enter normal operation mode.
During debug mode, all incoming external commands will be ignored.

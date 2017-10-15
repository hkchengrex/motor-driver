# Path

For all close loop operation, like position setting or velocity setting, a path will be generated for the PID to control. The path is theorized to be a trapezoidal shape, it has a constant acceleration and deceleration. The path mainly consist of three phases:
1. It accelerate to the terminal velocity
2. It maintain a constant velocity
3. It decelerate to the zero velocity

In case of velocity setting, there is no part 3. In case of position setting and the area under the curve (displacement) couldn't meet the desired value, the terminal velocity might be decreased to be lower than the max velocity set by the user.

## Feedback
Path arrival feedback will be emitted by the driver when the path has arrived at any of the three points:
1. When it has finished acceleration
2. When it has started deceleration
3. When it has finished deceleration, aka finished the entire path

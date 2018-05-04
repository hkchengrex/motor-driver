# Tweaking acceleration and velocity
This page and function is added on May 2018.

## The problem
It was known (but not investigated) that there exist integration error at the end of the path. The last output from the path generator usually differ by a few dozen encoder ticks with the actual target value. It was thought to be unavoidable numerical error (it is) and was solved using intermediate re-calibration point. That is, instead of simply forcing the path position to jump to the target position after the path has finished, we force the path position to update to ideal position at every critical time instances. This creates a discontinuous acceleration and non-differentiable velocity and harms the performance of our stupid fragile PID controller.

The problem is that all time steps are discrete values. A simple example is that it cannot accelerate to 5000 CNT/s with 2000 CNT/s^2 if our time step is 1s -- it would take 2.5 time steps to do so, but our time steps are discrete. Similarly, we cannot travel to 3000 CNT using 5000 CNT/s^2 acceleration in a triangular path. If we have to enforce the acceleration constraint, there will be position error. 

## Solution
### Triangular path
For triangular path, we have a tri\_vel which denotes the ideal maximum velocity it can achieve before it decelerate. It is decided to tweak this tri\_vel to be an integral multiple (in terms of time step) of acceleration. For the second half, the acceleration is tweaked so that the path will always end at the desired position with 0 velocity.

### Trapezium path
For trapezium path, the acceleration of the acceleration phase is tweaked. The change propagate to the second part, we then tweak the time taken by the constant velocity phase to be an integer. Finally the deceleration is tweaked in similar manner.

## Side effects
- Actual acceleration will be slightly smaller than or the same as the user-set acceleration. It will never exceed user-set acceleration.
- The maximum velocity achieved by triangular path might be a bit slower.
- Acceleration and deceleration might not be symmetrical.
- The whole path might takes 1 or 2 more time step(s).

All of these side effects should be negligible (I hope). Increasing the control frequency will reduce these errors.
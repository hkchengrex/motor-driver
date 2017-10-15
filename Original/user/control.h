#ifndef _CONTROL_H
#define _CONTROL_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "math.h"
#include "encoder.h"
#include "path.h"
#include "pwm.h"
#include "can_motor.h"
#include "define.h"
#include "path.h"

//PID value
extern volatile s32 curr_err;
extern volatile s32 diff_err;
extern volatile s32 inte_err;

extern volatile s32 last_set_vel;
extern volatile s32 last_set_pos;
extern volatile CloseLoopFlag last_set_flag;

extern volatile s32 max_accel, max_veloc;

//Initialize the control timer
void control_init(void);

/** A interface to generate a path using the path module
* @param pos: The position to stop at
*/
Path* setup_path(s32 pos);

//Enable/disable pid control
void set_loop_flag(CloseLoopFlag mflag);

//Enable pid control, and gen const vel
Path* set_close_loop_vel(s32 vel);

//Disable pid control, and set pwm
void set_open_loop_vel(s32 vel);

//Set max. velocity, unit: CNT/s
void set_max_vel(s32 vel);

//Set max. accleration: unit: CNT/s^2
void set_max_acc(s32 acc);

//Stop the motor in a dog-rush manner. No acceleration applied.
void sudden_stop(void);

//Set the competition mode to be true or false
void set_competition_mode(bool enable);

//Set the silence mode to be true or false
void set_stfu_mode(bool enable);

//Clear PID value
void pid_clear(void);

//Helper function to get values in their unit, CNT, CNT/s, CNT/s^2
s32 get_unit_vel(void);
s32 get_unit_acc(void);
s32 get_unit_cnt(void);
#endif

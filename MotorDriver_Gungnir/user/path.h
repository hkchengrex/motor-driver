#ifndef _PATH_H
#define _PATH_H

#include <stdbool.h>
#include "stm32f10x.h"
#include "math.h"
#include "encoder.h"
#include "uart.h"

typedef struct{
	s32 nom_acc; //Acceleration used in the path, same sign as first segment, cnt/s^-2
	s32 seg_acc; //Acceleration in the current segment, correct sign, cnt/s^-2
	s32 vt; //Terminal velocity, correct sign, cnt/s^-1
	s32 ve; //The velocity that it should maintain at the end of the path, cnt/s^-1
	s32 tar_vel; //Target velocity, cnt/s^-1
	s32 tar_pos; //Target position, cnt
	s32 tar_vel_r; //Target velocity remainder, cnt/s^-1
	s32 tar_pos_r; //Target position remainder, cnt
	u32 t1; //End of acceleration phase, in number of iteration
	u32 t2; //End of constant phase, in number of iteration
	u32 t3; //End of path, in number of iteration
	u32 itr; //The current number of iteration
	s32 t1_pt; //Position at t1, cnt, used to re-cali
	s32 t2_pt; //Position at t2, cnt, used to re-cali
	s32 end_pt; //Ending position, cnt, used to re-cali
	u8 dir; //Direction of path, based on the s0, sn and vt
} Path;

/** Iterate the path, updating path value, must be called with @CONTROL_FREQ
* @param enc_vel: The current unit velocity, CNT/s
* @param enc_pos: The current encoder position
*/
Path* path_iterate(void);

/** Generate a trapeziodal path to reach the destingated position, continuous with the current path
* @param sn: The final encoder position to stop at
* @param v_max: The max. velocity that can be used. UNSIGNED.
* @param acc: The acceleration to be used
*/
Path* gen_continuous_path(s32 sn, s32 v_max, s32 acc);

/** Generate a trapeziodal path to reach the destingated position
* @param v0: The starting unit velocity, CNT/s 
* @param s0: The starting encoder position
* @param vr: Velocity remainder
* @param sr: Encoder remainder
* @param sn: The final encoder position to stop at
* @param v_max: The max. velocity that can be used. UNSIGNED.
* @param acc: The acceleration to be used
*/
Path* gen_path(s32 v0, s32 s0, s32 vr, s32 sr, s32 sn, s32 v_max, s32 acc);

/** Generate a trapeziodal path to reach the destingated velocity and keep going with that velocity
* @param vt: The final velocity to stay at
* @param acc: The acceleration to be used
*/
Path* gen_continuous_const_vel(s32 vt, s32 acc);

/** Generate a trapeziodal path to reach the destingated velocity and keep going with that velocity
* @param v0: The The current unit velocity, CNT/s 
* @param s0: The current encoder position
* @param vr: Velocity remainder
* @param sr: Encoder remainder
* @param vt: The final velocity to stay at
* @param acc: The acceleration to be used
*/
Path* gen_const_vel(s32 v0, s32 s0, s32 vr, s32 sr, s32 vt, s32 acc);

//To check if the path has finished running or not [That it will not change velocity anymore]
bool is_path_running(void);

//Reset the path, lock the motor in position
void path_reset(void);

//Scaled by freq, remainder included
s32 get_path_vel_scaled(void);
s64 get_path_pos_scaled(void);

//Unscaled, remainder not included
s32 get_path_vel(void);
s32 get_path_pos(void);

//Get the current path parameters
u8 get_curr_path(void);
u32 get_t1(void);
u32 get_t2(void);
u32 get_t3(void);
s32 get_vt(void);
s32 get_ve(void);
u32 get_itr(void);
u8 get_path_dir(void);
bool get_next_required(void);
bool is_path_static(void);

#endif

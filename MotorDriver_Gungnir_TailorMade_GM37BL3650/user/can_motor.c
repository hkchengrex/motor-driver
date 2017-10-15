#include "can_motor.h"
#include "path.h"
#include "ticks.h"
#include "debug.h"
#include "control.h"

volatile u32 last_recv_ticks = 0;
u8 this_motor = THIS_MOTOR;

//Command decoding
static void motor_cmd_decoding(CanRxMsg* msg){
	
	if (is_debugging()){
		return;
	}
	
	last_recv_ticks = get_ticks();
	
	static u8 last_motion_command = GUN_CAN_MOTOR_VEL_CMD;
	
	switch (msg->Data[0]) {
		
		case GUN_CAN_MOTOR_VEL_CMD:
		
			if (msg->DLC == GUN_CAN_MOTOR_VEL_LENGTH) {
				s32 vel = msg->Data[1] | msg->Data[2] << 8 | msg->Data[3] << 16 | msg->Data[4] << 24;
				CloseLoopFlag flag = (CloseLoopFlag) msg->Data[5];
				
				static bool first_cmd = true;
				
				if (last_motion_command == GUN_CAN_MOTOR_VEL_CMD && !first_cmd && (last_set_vel == vel) && (last_set_flag == flag)){
					break;
				}
				first_cmd = false;
				last_motion_command = GUN_CAN_MOTOR_VEL_CMD;
				
				if (flag == CLOSE_LOOP){
					//Set up close loop velocity control
					set_close_loop_vel(vel);
				}else{
					//Set up open loop velocity control
					set_open_loop_vel(vel);
				}
			}
			break;
			
		case GUN_CAN_MOTOR_ACCEL_CMD:
			
			if (msg->DLC == GUN_CAN_MOTOR_ACCEL_LENGTH) {
				s32 accel = msg->Data[1] | msg->Data[2] << 8 | msg->Data[3] << 16 | msg->Data[4] << 24;
				
				static bool first_cmd = true;
				if (!first_cmd && (max_accel == accel)){
					break;
				}
				first_cmd = false;
				set_max_acc(accel);
			}
			break;
			
		case GUN_CAN_MOTOR_MAXV_CMD:
			
			if (msg->DLC == GUN_CAN_MOTOR_MAXV_LENGTH) {
				s32 velc = msg->Data[1] | msg->Data[2] << 8 | msg->Data[3] << 16 | msg->Data[4] << 24;
				
				static bool first_cmd = true;
				if (!first_cmd && (max_veloc == velc)){
					break;
				}
				first_cmd = false;
				set_max_vel(velc);
			}
			break;
			
		case GUN_CAN_MOTOR_POS_CMD:
			
			if (msg->DLC == GUN_CAN_MOTOR_POS_LENGTH) {
				s32 pos = msg->Data[1] | msg->Data[2] << 8 | msg->Data[3] << 16 | msg->Data[4] << 24;
				
				static bool first_cmd = true;
				if (last_motion_command == GUN_CAN_MOTOR_POS_CMD && !first_cmd && (pos == last_set_pos)){
					break;
				}
				first_cmd = false;
				last_motion_command = GUN_CAN_MOTOR_POS_CMD;
				setup_path(pos);
			}
			break;
			
		case GUN_CAN_MOTOR_LOCK_CMD:
			
			if (msg->DLC == GUN_CAN_MOTOR_LOCK_LENGTH) {
				static bool first_cmd = true;
				if (last_motion_command == GUN_CAN_MOTOR_LOCK_CMD && !first_cmd){
					break;
				}
				first_cmd = false;
				last_motion_command = GUN_CAN_MOTOR_LOCK_CMD;
				sudden_stop();
			}
			break;
			
		case GUN_CAN_MOTOR_COMPETITION_MODE_CMD:
			if (msg->DLC == GUN_CAN_MOTOR_COMPETITION_MODE_LENGTH) {
				bool competition_mode_enable = msg->Data[1]==ENABLE?1:0;
				set_competition_mode(competition_mode_enable);
			}
			break;
			
		case GUN_CAN_MOTOR_STFU_MODE_CMD:
			if (msg->DLC == GUN_CAN_MOTOR_STFU_MODE_LENGTH) {
				bool stfu_mode_enabled = msg->Data[1]==ENABLE?1:0;
				set_stfu_mode(stfu_mode_enabled);
			}
			break;
	}
}

void can_motor_init(){
	can_rx_add_filter(get_command_id(this_motor), CAN_RX_MASK_EXACT, 	motor_cmd_decoding);
}

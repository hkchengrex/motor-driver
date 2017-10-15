#ifndef _CAN_MOTOR_H
#define _CAN_MOTOR_H

#include "can_protocol.h"
#include "define.h"

#define get_command_id(motor_id)	(GUN_CAN_MOTOR_COMMAND_BASE + (uint8_t)motor_id)
#define get_feedback_id(motor_id)	(GUN_CAN_MOTOR_FEEDBACK_BASE + (uint8_t)motor_id)

extern volatile u32 last_recv_ticks;

#define GUN_CAN_MOTOR_COUNT								16

#define	GUN_CAN_MOTOR_COMMAND_BASE				0x0D0
#define	GUN_CAN_MOTOR_FEEDBACK_BASE				0x0E0

/*** TX ***/
#define	GUN_CAN_MOTOR_VEL_LENGTH					6
#define GUN_CAN_MOTOR_VEL_CMD							0xAA

#define GUN_CAN_MOTOR_POS_LENGTH 					5
#define GUN_CAN_MOTOR_POS_CMD 						0xBC

#define GUN_CAN_MOTOR_ACCEL_LENGTH				5
#define GUN_CAN_MOTOR_ACCEL_CMD						0x45

#define GUN_CAN_MOTOR_MAXV_LENGTH					5
#define GUN_CAN_MOTOR_MAXV_CMD						0x49

#define GUN_CAN_MOTOR_LOCK_LENGTH	 		  	1
#define GUN_CAN_MOTOR_LOCK_CMD						0xEE

#define GUN_CAN_MOTOR_COMPETITION_MODE_LENGTH	 		  	2
#define GUN_CAN_MOTOR_COMPETITION_MODE_CMD						0xDD

#define GUN_CAN_MOTOR_STFU_MODE_LENGTH	 		  	2
#define GUN_CAN_MOTOR_STFU_MODE_CMD						0xCC

/*** RX ***/
#define GUN_CAN_ENCODER_FEEDBACK_LENGTH		8
#define GUN_CAN_ENCODER_FEEDBACK_ID				0x22
#define GUN_CAN_ENCODER_OK_MASK						0x01
#define GUN_CAN_ENCODER_LM_LO_MASK				0x04
#define GUN_CAN_ENCODER_LM_HI_MASK				0x08

#define GUN_CAN_LIMIT_FEEDBACK_LENGTH 		5
#define GUN_CAN_LIMIT_LO_FEEDBACK			 		0x30
#define GUN_CAN_LIMIT_HI_FEEDBACK			 		0x31

#define GUN_CAN_ARRIVAL_FEEDBACK_LENGTH		1
#define GUN_CAN_ARRIVAL_PT_1_FEEDBACK			0x40 //End of acceleration
#define GUN_CAN_ARRIVAL_PT_2_FEEDBACK			0x41 //Beginning of deceleration
#define GUN_CAN_ARRIVAL_PT_3_FEEDBACK			0x42 //Entire path finished

typedef enum {
	MOTOR_1 = 0,
	MOTOR_2,
	MOTOR_3,
	MOTOR_4,
	MOTOR_5,
	MOTOR_6,
	MOTOR_7,
	MOTOR_8,
	MOTOR_9,
	MOTOR_10,
	MOTOR_11,
	MOTOR_12,
	MOTOR_13,
	MOTOR_14,
	MOTOR_15,
	MOTOR_16
} MotorID;

typedef enum{
	OPEN_LOOP = 0,
	CLOSE_LOOP = 1
} CloseLoopFlag;

void can_motor_init(void);

#endif

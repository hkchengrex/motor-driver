#ifndef _DEFINE_H
#define _DEFINE_H

// This is the motor id of the current motor
#define THIS_MOTOR MOTOR_4
extern u8 this_motor;

//Enable this if this poor motor somehow does not has a encoder
#define ENCODER_DISABLED false

//This can be 3 or 5
#define MOTOR_CONTROLLER_VERSION 5

#if (MOTOR_CONTROLLER_VERSION != 3) && (MOTOR_CONTROLLER_VERSION != 5)
	#error Motor controller version not supported
#endif

//These two options are for activating the high and low limit switches for lifting/angle tuning application
//When the switch is triggered, it will clear PID, do open loop zero, and emit signal to mainboard
//After that, only commands that move in the opposite direction as the limit switch will be respected.
#define ACTIVATE_HIGH_LIMIT_SWITCH	false
#define ACTIVATE_LOW_LIMIT_SWITCH		false
	
//For Motor controller 3.2 and 5.0, there are on-board pull-down resistors
//Most external encoders are using pull-up. In this case you need to remove the onboard resistors and change this to pull-up.

#define ENCODER_INPUT_MODE GPIO_Mode_IN_FLOATING

//Fuck You! Use 512! By Ben Wong
#define ENCODER_RESOLUTION 512

#if ENCODER_RESOLUTION==512 
	#define ENCODER_PRESCALER 1
#elif ENCODER_RESOLUTION==1024
	//Else is 1024
	#define ENCODER_PRESCALER 2
#else
	//Other resolution? You are drunk, go home
	#error Hi Today is a good day
#endif

#define CONTROL_FREQ 512
	
//Direction: 1 or -1
#define PWM_DIR (1)
#define ENC_DIR (1)

//Define this if you trust the PID!
//#define I_TRUST_PID
#define I_TRUST_MYSELF
//Note that this is useless now, but I'm keeping it because I wanted more trust and love in this world :)

#define MAX_ORIG_VEL (150000)
#define MAX_ORIG_ACC (300000)

#define MALF_FLASH_FREQ 42
#define NORM_FLASH_FREQ 256

#endif

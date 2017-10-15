#include "control.h"
#include "debug.h"
#include "limit.h"

#define PID_POS_KP 45643
#define PID_POS_KI 745
#define PID_POS_KD 294
#define PID_POS_KV 0//142

#define PID_NEG_KP 45643
#define PID_NEG_KI 745
#define PID_NEG_KD 294
#define PID_NEG_KV 0//142

#define PID_NEU_KP PID_POS_KP
#define PID_NEU_KI PID_POS_KI
#define PID_NEU_KD PID_POS_KD
#define PID_NEU_KV PID_POS_KV

#define PID_SCALE (1024)
#define MAX_I (CONTROL_FREQ*8)
#define MOSFET_REVERSE_COEFF 1/512

//Physical constants
volatile s32 max_accel = MAX_ORIG_ACC;
volatile s32 max_veloc = MAX_ORIG_VEL;

volatile s32 last_set_vel = 0;
volatile s32 last_set_pos = 0;
volatile CloseLoopFlag last_set_flag = OPEN_LOOP;
volatile static bool competition_mode_enabled = false;
volatile static bool stfu_mode_enabled = false;

inline s32 get_unit_vel(){
	return get_vel() * CONTROL_FREQ;
}

inline s32 get_unit_acc(){
	return get_acc() * CONTROL_FREQ * CONTROL_FREQ;
}

inline s32 get_unit_cnt(){
	return get_cnt();
}

Path* setup_path(s32 pos){
	//Where are you from?
	//Where are you going to?
	last_set_vel = 0;
	last_set_pos = pos;
	set_loop_flag(CLOSE_LOOP);
	return gen_continuous_path(pos, max_veloc, max_accel);
}

Path* set_close_loop_vel(s32 vel){
	last_set_vel = vel;
	set_loop_flag(CLOSE_LOOP);
	vel = CAP(vel, -max_veloc, max_veloc);
	return gen_continuous_const_vel(vel, max_accel);
}

void set_open_loop_vel(s32 vel){
	last_set_vel = vel;
	set_loop_flag(OPEN_LOOP);
	vel = CAP(vel, -MAX_PWM, MAX_PWM);
	set_pwm(vel);
}

void set_loop_flag(CloseLoopFlag mflag){
	if (last_set_flag != mflag){
		pid_clear();
		path_reset();
	}
	last_set_flag = mflag;
}

void set_max_vel(s32 vel){
	max_veloc = ABS(vel);
}

void set_max_acc(s32 acc){
	max_accel = ABS(acc);
}

void sudden_stop(){
	path_reset();
}

void set_competition_mode(bool enable){
	competition_mode_enabled = enable;
}

void set_stfu_mode(bool enable){
	//Personally I think the original frequency is better for hardware and helps debugging but well just stfu.
	if (stfu_mode_enabled != enable){
		pwm_init(enable);
		stfu_mode_enabled = enable;
	}
}

void control_init(){
	//If you think you're controlling it, you're wrong.
	encoder_init();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	//Timer init
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period 				= 100 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Prescaler 		= 72000000/CONTROL_FREQ/100 - 1;
	TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	//Interrupt init
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);  
	TIM_Cmd(TIM1, ENABLE);
	
	//Terribly wrong.
}

volatile s64 curr_err = 0;
volatile s64 diff_err = 0;
volatile s64 inte_err = 0;
volatile s64 velc_err = 0;
volatile s64 last_dif = 0;

s32 update_pid(){
	s64 curr_vel = get_unit_vel();
	curr_err = get_path_pos() - get_unit_cnt();
	diff_err = get_path_vel() - curr_vel;
	velc_err = diff_err - last_dif;
	last_dif = diff_err;
	
	inte_err = inte_err*99/100 + curr_err;
	if (inte_err > MAX_I){
		inte_err = MAX_I;
	}else if (inte_err < -MAX_I){
		inte_err = -MAX_I;
	}
	
	s64 Kp = 0;
	s64 Ki = 0;
	s64 Kd = 0;
	s64 Kv = 0;
	if (get_path_dir() == DIR_POS){
		//Uses up pid
		Kp = PID_POS_KP;
		Ki = PID_POS_KI;
		Kd = PID_POS_KD;
		Kv = PID_POS_KV;
	}else if(get_path_dir() == DIR_NEG){
		//Uses down pid
		Kp = PID_NEG_KP;
		Ki = PID_NEG_KI;
		Kd = PID_NEG_KD;
		Kv = PID_NEG_KV;
	}else{
		//Use default set
		Kp = PID_NEU_KP;
		Ki = PID_NEU_KI;
		Kd = PID_NEU_KD;
		Kv = PID_NEU_KV;
	}
	
	s64 duty = (s64)(Kp*curr_err + Ki*inte_err + Kd*diff_err + Kv*velc_err)/(s64)PID_SCALE;
	
	//This section helps reduce tak tak sound
	if ((duty!=0) && (ABS(curr_vel)>4096) && (SIGN(curr_vel) != SIGN(duty))){
		//If the mosfet direction is opposite to the traveling direction
		s64 offset = ABS(curr_vel)*MOSFET_REVERSE_COEFF;
		if (duty > 0){
			s64 offseted = duty - offset;
			if (offseted > 0){
				duty = offseted;
			}else{
				duty = 1;
			}
		}else{
			s64 offseted = duty + offset;
			if (offseted < 0){
				duty = offseted;
			}else{
				duty = -1;
			}
		}
	}
	
	if (duty > MAX_PWM*MAX_POWER_PERCENTAGE/100){
		duty = MAX_PWM;
	}else if(duty < -MAX_PWM*MAX_POWER_PERCENTAGE/100){
		duty = -MAX_PWM;
	}
	
	return duty;
}

void pid_clear(){
	curr_err = 0;
	diff_err = 0;
	inte_err = 0;
	velc_err = 0;
	last_dif = 0;
}

static u32 encoder_malf_cnt = 0;
//Called at @CONTROL_FREQ Hz
void TIM1_UP_IRQHandler(){
	
	if (TIM_GetITStatus(TIM1, TIM_FLAG_Update)){
		TIM_ClearITPendingBit(TIM1, TIM_FLAG_Update);
		
		//Update encoder count/vel/acc at @CONTROL_FREQ
		limit_update(last_set_flag);
		encoder_update();
		
		if (last_set_flag == CLOSE_LOOP){
			s32 pwm = 0;
			
			if (!is_debugging() && encoder_malfunction()){
				encoder_malf_cnt = (encoder_malf_cnt+1) % (CONTROL_FREQ/4);
				pid_clear();
				path_reset();
				
				if (competition_mode_enabled){
					//Map close loop to open loop so that the robot is still usable in competition
					
					//This is a mini controller for pure pwm control
					static Direction pwm_dir = DIR_POS;
					static s32 last_pwm = 0;
					const static s32 pwm_accel = 512;
					pwm = (s32)((s64)last_set_vel*(s64)MAX_PWM/(s64)max_veloc);
					
					//Acceleration
					if (pwm > last_pwm+pwm_accel){
						pwm = last_pwm+pwm_accel;
					}else if(pwm < last_pwm-pwm_accel){
						pwm = -last_pwm-pwm_accel;
					}
					
					if (pwm == 0){
						//Hardware braking section
						if (pwm_dir == DIR_POS){
							pwm = -1;
						}else{
							pwm = 1;
						}
						
					}else{
						//Normal condition
						if (pwm > 0){
							pwm_dir = DIR_POS;
						}else if(pwm < 0){
							pwm_dir = DIR_NEG;
						}
					}
					
					last_pwm = pwm;
					
				}else{
					//Competition mode not enabled. Enter auto recovery routine
					last_set_vel = 0;
					
					//Auto recovery
					if (encoder_malf_cnt == CONTROL_FREQ/64){
						pwm = (MAX_PWM/8);
					}else if(encoder_malf_cnt == CONTROL_FREQ/128){
						pwm = (-MAX_PWM/8);
					}else{
						pwm = 0;
					}
				}
				
			}else{
				encoder_malf_cnt = 0;
				path_iterate();
				pwm = update_pid();
			}
			
			set_pwm(pwm);
		}
	}
}

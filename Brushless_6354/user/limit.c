#include "limit.h"
#include "feedback.h"
#include "path.h"
#include "pwm.h"
#include "control.h"

void limit_init(){
	GPIO_InitTypeDef GPIO_InitStructure;
	
	LM_HI_RCC_init();
	LM_LO_RCC_init();
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_InitStructure.GPIO_Mode = LM_INPUT_MODE;
	
	GPIO_InitStructure.GPIO_Pin = LM_HI_Pin;
  GPIO_Init(LM_HI_GPIOx, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = LM_LO_Pin;
	GPIO_Init(LM_LO_GPIOx, &GPIO_InitStructure);
}

u8 get_limit(LIMIT_ID id){
	if (LM_INPUT_MODE == GPIO_Mode_IPD){
		if (id == LIMIT_HI){
			#if ACTIVATE_HIGH_LIMIT_SWITCH
			return GPIO_ReadInputDataBit(LM_HI_GPIOx, LM_HI_Pin);
			#else
			return 0;
			#endif
		}else if(id == LIMIT_LO){
			#if ACTIVATE_LOW_LIMIT_SWITCH
			return GPIO_ReadInputDataBit(LM_LO_GPIOx, LM_LO_Pin);
			#else
			return 0;
			#endif
		}
	}else{
		if (id == LIMIT_HI){
			#if ACTIVATE_HIGH_LIMIT_SWITCH
			return !GPIO_ReadInputDataBit(LM_HI_GPIOx, LM_HI_Pin);
			#else
			return 0;
			#endif
		}else if(id == LIMIT_LO){
			#if ACTIVATE_LOW_LIMIT_SWITCH
			return !GPIO_ReadInputDataBit(LM_LO_GPIOx, LM_LO_Pin);
			#else
			return 0;
			#endif
		}
	}
	return 0;
}

void limit_update(CloseLoopFlag flag){
	static u8 last_limit[2] = {2, 2};
	static s32 limit_boundary_enc[2] = {0, 0};

	u8 curr_limit[2];
	curr_limit[0] = get_limit(LIMIT_LO);
	curr_limit[1] = get_limit(LIMIT_HI);
	
	for (LIMIT_ID i=LIMIT_LO; i<=LIMIT_HI; i++){
		if (curr_limit[i]){
			if (last_limit[i] == 0){
				limit_boundary_enc[i] = get_unit_cnt();
				yield_limit_feedack(i, limit_boundary_enc[i]);
				
				if (flag == CLOSE_LOOP){
					setup_path(limit_boundary_enc[i]);
				}else{
					set_open_loop_vel(0);
				}
			}
			
			if (flag == CLOSE_LOOP){
				if (i == LIMIT_LO){
					if (get_path_dir() == DIR_NEG){
						//If a new path has been set that tries to break shit, invalidate it and tell it who is the boss here
						setup_path(limit_boundary_enc[i]);
					}
				}else{
					if (get_path_dir() == DIR_POS){
						//If a new path has been set that tries to break shit, invalidate it and tell it who is the boss here
						setup_path(limit_boundary_enc[i]);
					}
				}
			}
		}else{
			if (last_limit[i] == 1){
				limit_boundary_enc[i] = get_unit_cnt();
				yield_limit_feedack(i, limit_boundary_enc[i]);
			}
		}
		last_limit[i] = curr_limit[i];
	}
}

#ifndef _FEEDBACK_H
#define _FEEDBACK_H

#include "stm32f10x.h"
#include "uart.h"
#include "can_motor.h"
#include "encoder.h"
#include "limit.h"

#define UART_FEEDBACK_COM COM1

//Send feedback via UART
void uart_feedback(void);

//Send encoder feedback via CAN
void can_feedback(bool enc_working);

//Send limit switch feedback via CAN
void yield_limit_feedack(LIMIT_ID id, s32 enc);

//Send point arrival signal via CAN
void pt_arrival_feedback(u8 point);

//Control the uart feedback output
void uart_feedback_shut(void);
void uart_feedback_live(void);

#endif

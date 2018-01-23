#ifndef _CAN_PROTOCOL_H
#define _CAN_PROTOCOL_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"
#include "misc.h"
#include <stdbool.h>

#define	CANn							CAN1
#define CAN_RCC						RCC_APB1Periph_CAN1

#define	CAN_Rx_GPIO_Pin		GPIO_Pin_11
#define	CAN_Tx_GPIO_Pin		GPIO_Pin_12
#define	CAN_GPIO_PORT			GPIOA
#define CAN_GPIO_RCC			RCC_APB2Periph_GPIOA

#define CAN_TX_QUEUE_MAX_SIZE				50
#define	CAN_TX_IRQn									USB_HP_CAN1_TX_IRQn
#define CAN_TX_IRQHander						USB_HP_CAN1_TX_IRQHandler

#define CAN_Rx_IRQn									USB_LP_CAN1_RX0_IRQn
#define	CAN_Rx_IRQHandler						USB_LP_CAN1_RX0_IRQHandler
#define	CAN_RX_FILTER_LIMIT		28		//Max. number of filter


/*** X = the ID bit that must be equal 	***/
/*** ? = the ID bit that can varies 		***/
																						/***   11-bit ID   	(example range) 		***/
#define CAN_RX_MASK_EXACT						0x7FF		/*** XXX XXXX XXXX	(Exactly same ID)		***/
#define CAN_RX_MASK_DIGIT_0_F				0x7F0		/*** XXX XXXX ???? 	(0xAB0 - 0xABF) 		***/
#define	CAN_RX_MASK_DIGIT_0_7				0x7F8		/*** XXX XXXX X??? 	(0xAB0 - 0xAB7)			***/
#define	CAN_RX_MASK_DIGIT_0_3				0x7FC		/*** XXX XXXX XX?? 	(0xAB0 - 0xAB3)			***/
#define	CAN_RX_MASK_DIGIT_0_1				0x7FE		/*** XXX XXXX XXX? 	(0xAB0 - 0xAB1)			***/


typedef struct{
	uint16_t id;					// 11-bit ID: 0x000 ~ 0x7FF
	uint8_t length;			// 0 ~ 8
	uint8_t data[8];
}CanMessage;

typedef struct{
	volatile uint16_t head;
	volatile uint16_t tail;
	CanMessage* queue;
}CanQueue;

void can_init(void);

bool can_mailbox_empty(void);
bool can_tx_enqueue(CanMessage msg);
void can_tx_dequeue(void);
void can_tx_queue_clear(void);	

/*** CAN Rx ***/
void can_rx_init(void);
void can_rx_add_filter(u16 id, u16 mask, void (*handler)(CanRxMsg *msg));

#endif

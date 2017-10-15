#include "can_protocol.h"
#include "uart.h"
#include "feedback.h"

CanMessage CANTxQueueArray[CAN_TX_QUEUE_MAX_SIZE];
CanQueue CAN_tx_queue = {0, 0, CANTxQueueArray};
u8 CAN_FilterCount = 0;

void (*CAN_Rx_Handlers[CAN_RX_FILTER_LIMIT])(CanRxMsg *msg);

void can_init(void){
	GPIO_InitTypeDef GPIO_InitStructure;
	CAN_InitTypeDef CAN_InitStructure;
	
	/* RCC enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(CAN_RCC, ENABLE);
	RCC_APB2PeriphClockCmd(CAN_GPIO_RCC, ENABLE);

	/* CAN GPIO init */
	// CAN_Rx Pin
	GPIO_InitStructure.GPIO_Pin = CAN_Rx_GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);
	
	// CAN_Tx Pin
	GPIO_InitStructure.GPIO_Pin = CAN_Tx_GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);
	
	/* CAN register init */
	CAN_DeInit(CANn);
	CAN_StructInit(&CAN_InitStructure);
	
	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;
	CAN_InitStructure.CAN_ABOM = ENABLE;
	CAN_InitStructure.CAN_AWUM = DISABLE;
	CAN_InitStructure.CAN_NART = DISABLE;
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = DISABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	
	/* CAN Baudrate = 1 MBPS */
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 6;
	CAN_Init(CANn, &CAN_InitStructure);
	
	/* CAN Transmission Mailbox Empty interrupt enable */ 
	CAN_ITConfig(CANn, CAN_IT_TME, ENABLE);
	
	/* CAN TX interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = CAN_TX_IRQn; 

	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
* Return the current size of the queue.
* Range: 0 ~ CAN_TX_QUEUE_MAX_SIZE-1
*/
uint16_t get_can_queue_size(){
	return (CAN_tx_queue.tail - CAN_tx_queue.head + CAN_TX_QUEUE_MAX_SIZE) % CAN_TX_QUEUE_MAX_SIZE;
}

bool can_tx_enqueue(CanMessage msg){
	if (get_can_queue_size() == (CAN_TX_QUEUE_MAX_SIZE - 1)){
		return false;
	}

	CAN_tx_queue.queue[CAN_tx_queue.tail] = msg;
	CAN_tx_queue.tail = (CAN_tx_queue.tail + 1) % CAN_TX_QUEUE_MAX_SIZE;

	can_tx_dequeue();
	return true;
}

void can_tx_dequeue(void){
	if (get_can_queue_size() && (CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
		#define msg CAN_tx_queue.queue[CAN_tx_queue.head]

    /* Set up the Id */
    CAN1->sTxMailBox[0].TIR &= ((uint32_t)0x00000001);

    CAN1->sTxMailBox[0].TIR |= (msg.id << 21) | (CAN_RTR_Data);
		
    /* Set up the DLC */
    CAN1->sTxMailBox[0].TDTR &= (uint32_t)0xFFFFFFF0;
    CAN1->sTxMailBox[0].TDTR |= msg.length;

    /* Set up the data field */
    CAN1->sTxMailBox[0].TDLR = (((uint32_t)msg.data[3] << 24) | 
                                             ((uint32_t)msg.data[2] << 16) |
                                             ((uint32_t)msg.data[1] << 8) | 
                                             ((uint32_t)msg.data[0]));
    CAN1->sTxMailBox[0].TDHR = (((uint32_t)msg.data[7] << 24) | 
                                             ((uint32_t)msg.data[6] << 16) |
                                             ((uint32_t)msg.data[5] << 8) |
                                             ((uint32_t)msg.data[4]));
    /* Request transmission */
    CAN1->sTxMailBox[0].TIR |= (uint32_t)0x00000001;
		
		CAN_tx_queue.head = (CAN_tx_queue.head + 1) % CAN_TX_QUEUE_MAX_SIZE;
		#undef msg
	}
	//uart_tx(UART_FEEDBACK_COM, "%d %d %d\n", CAN_tx_queue.head, CAN_tx_queue.tail, get_can_queue_size());
}

/**
	* @brief Force clear the CAN_TX queue without process
	* @param None.
	* @retval None.
	*/
void can_tx_queue_clear(void){
	CAN_tx_queue.head = CAN_tx_queue.tail = 0;
}


/**
	* @brief The handler function of empty 
	*/
void CAN_TX_IRQHander(){
	if (CAN_GetITStatus(CAN1, CAN_IT_TME) != RESET){
		// If all the mailboxes are empty
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
		can_tx_dequeue();
	}
}

/**
	* @brief Initialize the CAN_RX interrupt handler
	* @param None.
	* @retval None.
	*/
void can_rx_init(void){
	NVIC_InitTypeDef NVIC_InitStructure;
	
	CAN_ITConfig(CANn, CAN_IT_FMP0, ENABLE);

	/* enabling interrupt */
	NVIC_InitStructure.NVIC_IRQChannel= CAN_Rx_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
	* @brief Add filter to the can data received (involves bitwise calculation)
	* @warning can only be called for 14 / 28 times. Check the function IS_CAN_FILTER_NUMBER for detail
	* @param id: 11-bit ID (0x000 to 0x7FF)
	* @param mask: 11-bit mask, corresponding to the 11-bit ID	(0x000 to 0x7FF)		
	* @param handler: function pointer for the corresponding CAN ID filter
	* @example can_rx_add_filter(0x000, 0x000) will receive CAN message with ANY ID
	* @example can_rx_add_filter(0x0CD, 0x7FF) will receive CAN message with ID 0xCD
	* @example can_rx_add_filter(0x0A0, 0x7F0) will receive CAN message with ID from 0xA0 to 0xAF
	* @example can_rx_add_filter(0x000, 0x7FA) will receive CAN message with ID from 0x00 to 0x03
	*/
void can_rx_add_filter(u16 id, u16 mask, void (*handler)(CanRxMsg *msg)){
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	mask = ((mask << 5) | 0x001F) & 0xFFFF;
	
	CAN_FilterInitStructure.CAN_FilterNumber = CAN_FilterCount;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = (id << 5) & 0xFFFF;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = mask;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);	
	
	CAN_Rx_Handlers[CAN_FilterCount] = handler;
	
	++CAN_FilterCount;
}

/** 
	* @brief Interrupt for CAN Rx
	* @warning Use USB_LP_CAN_RX0_IRQHandler for HD, USB_LP_CAN1_RX0_IRQHandler for XLD / MD
	*/
void CAN_Rx_IRQHandler(){
	if (CAN_GetITStatus(CANn, CAN_IT_FMP0) != RESET) {
		CanRxMsg RxMessage;
		CAN_ClearITPendingBit(CANn, CAN_IT_FMP0);
		CAN_Receive(CANn, CAN_FIFO0, &RxMessage);
		
		if(RxMessage.IDE == CAN_ID_STD) {
			u8 filter_id = RxMessage.FMI;
			if (CAN_Rx_Handlers[filter_id] != 0 && filter_id < CAN_FilterCount) {
				CAN_Rx_Handlers[filter_id](&RxMessage);
			}
		}
	}
}

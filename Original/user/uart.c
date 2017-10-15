#include "uart.h"

USART_TypeDef* COM_USART[COMn] = {USART1, USART2, USART3}; 
GPIO_TypeDef* COM_TX_PORT[COMn] = {COM1_TX_GPIO_PORT, COM2_TX_GPIO_PORT, COM3_TX_GPIO_PORT}; 
GPIO_TypeDef* COM_RX_PORT[COMn] = {COM1_RX_GPIO_PORT, COM2_RX_GPIO_PORT, COM3_RX_GPIO_PORT}; 
uc32 COM_USART_CLK[COMn] = {COM1_CLK, COM2_CLK, COM3_CLK};
uc32 COM_TX_PORT_CLK[COMn] = {COM1_TX_GPIO_CLK, COM2_TX_GPIO_CLK, COM3_TX_GPIO_CLK}; 
uc32 COM_RX_PORT_CLK[COMn] = {COM1_RX_GPIO_CLK, COM2_RX_GPIO_CLK, COM3_RX_GPIO_CLK};
uc16 COM_TX_PIN[COMn] = {COM1_TX_PIN, COM2_TX_PIN, COM3_TX_PIN};
uc16 COM_RX_PIN[COMn] = {COM1_RX_PIN, COM2_RX_PIN, COM3_RX_PIN};
uc16 COM_IRQ[COMn] = {USART1_IRQn, USART2_IRQn, USART3_IRQn};

static volatile uint8_t tx_buf_item_1[UART1_TX_BUFFER_MAX] = {0};
static volatile uint8_t tx_buf_item_2[UART2_TX_BUFFER_MAX] = {0};
static volatile uint8_t tx_buf_item_3[UART3_TX_BUFFER_MAX] = {0};

static volatile UartQueue tx_queue[COMn] = {{0, 0, tx_buf_item_1}, 
																				{0, 0, tx_buf_item_2},
																				{0, 0, tx_buf_item_3}};
static uint32_t tx_buf_max_size[COMn] = {UART1_TX_BUFFER_MAX, UART2_TX_BUFFER_MAX, UART3_TX_BUFFER_MAX};

static OnRxListener rxListeners[COMn] = {0};

/**
  * @brief  Inintialization of USART
  * @param  COM: which USART to inialialize
  * @param  br: Baudrate used for USART
  * @retval None
  */
void uart_init(SerialPort COM, u32 br)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM] | RCC_APB2Periph_AFIO, ENABLE);

	if (COM == COM1)
	{
		RCC_APB2PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	}
	else
	{
		RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
	}

	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
	GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

	/* USART configuration */
	USART_InitStructure.USART_BaudRate = br;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_ITConfig(COM_USART[COM], USART_IT_TC, ENABLE);
	USART_ITConfig(COM_USART[COM], USART_IT_RXNE, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = COM_IRQ[COM];
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_OverSampling8Cmd(COM_USART[COM], ENABLE);
	USART_Init(COM_USART[COM], &USART_InitStructure);
	USART_Cmd(COM_USART[COM], ENABLE);
}

void uart_interrupt_init(SerialPort COM, OnRxListener listener){
	rxListeners[COM] = listener;
}

uint16_t get_uart_queue_size(SerialPort COM){
	return (tx_queue[COM].tail - tx_queue[COM].head + tx_buf_max_size[COM]) % tx_buf_max_size[COM];
}

/** Try to dequeue and transmit a btye
*		@param COM: Which port to use
*/
__INLINE static void uart_tx_dequeue(SerialPort COM){
	if (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TXE) == SET){
		if (get_uart_queue_size(COM) != 0){
			//Put into the data register
			COM_USART[COM]->DR = tx_queue[COM].queue[tx_queue[COM].head];
			
			//Dequeue
			tx_queue[COM].head = (tx_queue[COM].head + 1) % tx_buf_max_size[COM];
		}
	}
}

/** Enqueue a byte, ready to be transmitted
*		@param COM: Which port to use
*/
__INLINE static void uart_tx_enqueue(SerialPort COM, uint8_t data){
	if (get_uart_queue_size(COM) == tx_buf_max_size[COM]){
		return;
	}
	tx_queue[COM].queue[tx_queue[COM].tail] = data;
	tx_queue[COM].tail = (tx_queue[COM].tail + 1) % tx_buf_max_size[COM];
	
	uart_tx_dequeue(COM);
}

void uart_tx_byte_blocking(SerialPort COM, const char data){
	while (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TXE) == RESET); 
	USART_SendData(COM_USART[COM],data);
}

void uart_tx_blocking(SerialPort COM, const char * tx_buf, ...){
	va_list arglist;
	u8 buf[255], *fp;
	
	va_start(arglist, tx_buf);
	vsprintf((char*)buf, (const char*)tx_buf, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp)
		uart_tx_byte_blocking(COM,*fp++);
}

/** Send a single byte to the target port. Non-Blocking.
*		@param COM: Which port to use
*		@param data: The content to be sent
*/
__INLINE void uart_tx_byte(SerialPort COM, uint8_t data){
	if (USART_GetFlagStatus(COM_USART[COM], USART_FLAG_TXE) == SET && get_uart_queue_size(COM) == 0){
		uart_tx_byte_blocking(COM, data);
	}else{
		uart_tx_enqueue(COM, data);
	}
}

/** Send multiple bytes to the target port. Non-Blocking.
*		@param COM: Which port to use
*		@param data: The content to be sent
*/
void uart_tx(SerialPort COM, const char * data, ...){
	va_list arglist;
	uint8_t buf[512], *fp;
	
	va_start(arglist, data);
	vsprintf((char*)buf, data, arglist);
	va_end(arglist);
	
	fp = buf;
	while (*fp){
		uart_tx_enqueue(COM, *fp++);
	}
}

/** Send an array of data to the target port. Non-Blocking.
*		@param COM: Which port to use
*		@param data: The pointer to the first element
*		@param len: Length of the array (in bytes)
*/
void uart_tx_array(SerialPort COM, const uint8_t * data, u16 len){
	while(len--){
		uart_tx_enqueue(COM, *data++);
	}
}

#define COM_PORT COM1
void USART1_IRQHandler(void){
	if(USART_GetITStatus(COM_USART[COM_PORT], USART_IT_RXNE) == SET){
		//Handle receive interrupt
		if (rxListeners[COM_PORT] != 0){
			(rxListeners[COM_PORT])(USART_ReceiveData(COM_USART[COM_PORT]));
		}
		USART_ClearITPendingBit(COM_USART[COM_PORT], USART_IT_RXNE);
		
	}else if(USART_GetITStatus(COM_USART[COM_PORT], USART_IT_TC) == SET){
		//Handle transmit interrupt
		uart_tx_dequeue(COM_PORT);
		USART_ClearITPendingBit(COM_USART[COM_PORT], USART_IT_TC);
	}
	COM_USART[COM_PORT]->SR;
	COM_USART[COM_PORT]->DR;
}
#undef COM_PORT

#define COM_PORT COM2
void USART2_IRQHandler(void){
	if(USART_GetITStatus(COM_USART[COM_PORT], USART_IT_RXNE) == SET){
		//Handle receive interrupt
		if (rxListeners[COM_PORT] != 0){
			(rxListeners[COM_PORT])(USART_ReceiveData(COM_USART[COM_PORT]));
		}
		USART_ClearITPendingBit(COM_USART[COM_PORT], USART_IT_RXNE);
		
	}else if(USART_GetITStatus(COM_USART[COM_PORT], USART_IT_TC) == SET){
		//Handle transmit interrupt
		uart_tx_dequeue(COM_PORT);
		USART_ClearITPendingBit(COM_USART[COM_PORT], USART_IT_TC);
	}
	COM_USART[COM_PORT]->SR;
	COM_USART[COM_PORT]->DR;
}
#undef COM_PORT

#define COM_PORT COM3
void USART3_IRQHandler(void){
	if(USART_GetITStatus(COM_USART[COM_PORT], USART_IT_RXNE) == SET){
		//Handle receive interrupt
		if (rxListeners[COM_PORT] != 0){
			(rxListeners[COM_PORT])(USART_ReceiveData(COM_USART[COM_PORT]));
		}
		USART_ClearITPendingBit(COM_USART[COM_PORT], USART_IT_RXNE);
		
	}else if(USART_GetITStatus(COM_USART[COM_PORT], USART_IT_TC) == SET){
		//Handle transmit interrupt
		uart_tx_dequeue(COM_PORT);
		USART_ClearITPendingBit(COM_USART[COM_PORT], USART_IT_TC);
	}
	COM_USART[COM_PORT]->SR;
	COM_USART[COM_PORT]->DR;
}
#undef COM_PORT

#ifndef _UART_H
#define _UART_H

#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"

#include <stdio.h>
#include <stdarg.h>

typedef enum {
    COM1 = 0,
    COM2 = 1,
    COM3 = 2
} SerialPort;

#define COMn 3

// Definition for USART1 
#define COM1_CLK                    RCC_APB2Periph_USART1
#define COM1_TX_PIN                 GPIO_Pin_9
#define COM1_TX_GPIO_PORT           GPIOA
#define COM1_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define COM1_RX_PIN                 GPIO_Pin_10
#define COM1_RX_GPIO_PORT           GPIOA
#define COM1_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define COM1_IRQn                   USART1_IRQn

// Definition for USART2 
#define COM2_CLK                    RCC_APB1Periph_USART2
#define COM2_TX_PIN                 GPIO_Pin_2
#define COM2_TX_GPIO_PORT           GPIOA
#define COM2_TX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define COM2_RX_PIN                 GPIO_Pin_3
#define COM2_RX_GPIO_PORT           GPIOA
#define COM2_RX_GPIO_CLK            RCC_APB2Periph_GPIOA
#define COM2_IRQn                   USART2_IRQn

// Definition for USART3 
#define COM3_CLK                    RCC_APB1Periph_USART3
#define COM3_TX_PIN                 GPIO_Pin_10
#define COM3_TX_GPIO_PORT           GPIOB
#define COM3_TX_GPIO_CLK            RCC_APB2Periph_GPIOB
#define COM3_RX_PIN                 GPIO_Pin_11
#define COM3_RX_GPIO_PORT           GPIOB
#define COM3_RX_GPIO_CLK            RCC_APB2Periph_GPIOB
#define COM3_IRQn                   USART3_IRQn

#define UART1_TX_BUFFER_MAX 1024
#define UART2_TX_BUFFER_MAX 10
#define UART3_TX_BUFFER_MAX 200
typedef struct{
	volatile uint16_t head;
	volatile uint16_t tail;
	volatile uint8_t* queue;
}UartQueue;

typedef void (*OnRxListener)(const uint8_t byte);

void uart_init(SerialPort COM, u32 br);
void uart_interrupt_init(SerialPort COM, OnRxListener listener);

void uart_tx_byte_blocking(SerialPort COM, const char data);
void uart_tx_blocking(SerialPort COM, const char * tx_buf, ...);

/**
* The followings are NON-blocking TX functions.
*/


/** Send a single byte to the target port. Non-Blocking.
*		@param COM: Which port to use
*		@param data: The content to be sent
*/
void uart_tx_byte(SerialPort COM, uint8_t data);

/** Send multiple bytes to the target port. Non-Blocking.
*		@param COM: Which port to use
*		@param data: The content to be sent
*/
void uart_tx(SerialPort COM, const char * data, ...);

/** Send an array of data to the target port. Non-Blocking.
*		@param COM: Which port to use
*		@param data: The pointer to the first element
*		@param len: Length of the array (in bytes)
*/
void uart_tx_array(SerialPort COM, const uint8_t * data, uint16_t len);

#endif

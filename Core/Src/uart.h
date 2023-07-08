/*
 * uart.h
 *
 *  Created on: Jun 16, 2023
 *      Author: dkalaitzakis
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include "stm32l4xx_hal.h"

#define STR 0x02
#define STP 0x03
#define ESC 0x1B


typedef enum{
	UART_DEBUG,
	UART_NYX,
	UART_IRIS
}UART_select;

void MX_USART1_UART_Init(void);

void MX_UART4_Init(void);

HAL_StatusTypeDef uart_write_debug(uint8_t *pData, uint32_t Timeout);

HAL_StatusTypeDef uart_write(uint8_t *pData, UART_select device, uint32_t Timeout);



#endif /* SRC_UART_H_ */

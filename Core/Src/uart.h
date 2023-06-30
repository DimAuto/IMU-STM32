/*
 * uart.h
 *
 *  Created on: Jun 16, 2023
 *      Author: dkalaitzakis
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include "stm32l4xx_hal.h"

void MX_USART1_UART_Init(void);

void MX_UART4_Init(void);

HAL_StatusTypeDef uart_write_debug(uint8_t *pData, uint32_t Timeout);

HAL_StatusTypeDef uart_write_uart4(uint8_t *pData, uint32_t Timeout);



#endif /* SRC_UART_H_ */

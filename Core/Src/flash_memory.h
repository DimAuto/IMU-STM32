/*
 * flash_memory.h
 *
 *  Created on: Aug 1, 2023
 *      Author: dkalaitzakis
 */

#include "main.h"

#ifndef SRC_FLASH_MEMORY_H_
#define SRC_FLASH_MEMORY_H_


void FlashReadData (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);

void Flash_Write_NUM (uint32_t StartSectorAddress, float Num);

float Flash_Read_NUM (uint32_t StartSectorAddress);

uint32_t FlashWriteData (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);

#endif /* SRC_FLASH_MEMORY_H_ */

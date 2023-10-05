/*
 * flash_memory.h
 *
 *  Created on: Aug 1, 2023
 *      Author: dkalaitzakis
 */

#include "main.h"
#include "lsm6_gyro.h"

#ifndef SRC_FLASH_MEMORY_H_
#define SRC_FLASH_MEMORY_H_


void FlashReadData (uint32_t StartPageAddress, uint64_t *RxBuf, uint16_t numberofwords);

void Flash_Write_NUM (uint32_t StartSectorAddress, float Num);

float Flash_Read_NUM (uint32_t StartSectorAddress);

uint32_t FlashWriteData (uint32_t StartPageAddress, uint64_t *Data, uint16_t numberofwords);

uint32_t Flash_Write_CalTable (uint32_t StartSectorAddress, gyro_data_t *gyro_data);

uint32_t Flash_isWritten (uint32_t StartSectorAddress);

uint32_t FlashErase(uint32_t StartPageAddress, uint32_t numberofpages);

uint32_t Flash_Read_CalTable (uint32_t StartSectorAddress, gyro_data_t *data);

#endif /* SRC_FLASH_MEMORY_H_ */

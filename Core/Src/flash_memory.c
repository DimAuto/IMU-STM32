/*
 * flash_memory.c
 *
 *  Created on: Aug 1, 2023
 *      Author: dkalaitzakis
 */


#include "flash_memory.h"
#include "uart.h"
//#include "Fusion/Fusion.h"




static uint32_t GetPage(uint32_t Address)
{
  for (int indx=0; indx<256; indx++)
  {
	  if((Address < (FLASH_BASE + (FLASH_PAGE_SIZE *(indx+1))) ) && (Address >= (FLASH_BASE + FLASH_PAGE_SIZE*indx)))
	  {
		  return (FLASH_BASE + FLASH_PAGE_SIZE*indx);
	  }
  }
  return 0;
}

void float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }

}

float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}



void FlashReadData (uint32_t StartPageAddress, uint64_t *RxBuf, uint16_t numberofwords)
{
	while (1)
	{

		*RxBuf = *(__IO uint64_t *)StartPageAddress;
		StartPageAddress += 8;
		RxBuf++;
		if (!(numberofwords--)){
			break;
		}
	}
}


uint32_t FlashWriteData (uint32_t StartPageAddress, uint64_t *Data, uint16_t numberofwords)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar=0;

	  /* Unlock the Flash to enable the flash control register access *************/
	   HAL_FLASH_Unlock();

	   /* Erase the user Flash area*/

	  uint32_t StartPage = GetPage(StartPageAddress);
	  uint32_t EndPageAdress = StartPageAddress + numberofwords * 8;
	  uint32_t EndPage = GetPage(EndPageAdress);

	   /* Fill EraseInit structure*/
	   EraseInitStruct.Banks = FLASH_BANK_2;
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	   EraseInitStruct.Page = ((StartPage - FLASH_BASE) / FLASH_PAGE_SIZE) + 1;
	   EraseInitStruct.NbPages = ((EndPage - StartPage)/FLASH_PAGE_SIZE) + 1;

	   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
	   {
	     /*Error occurred while page erase.*/
		   uart_write_debug("Failed to erase flash\r\n",UART_NYX);
		  return HAL_FLASH_GetError ();

	   }

	   /* Program the user Flash area word by word*/

	   while (sofar<numberofwords)
	   {
	     if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, Data[sofar]) == HAL_OK)
	     {
	    	 StartPageAddress += 8;  // use StartPageAddress += 2 for half word and 8 for double word
	    	 sofar++;
	     }
	     else
	     {
	       /* Error occurred while writing data in Flash memory*/
	    	 uart_write_debug("Failed to write flash\r\n",UART_NYX);
	    	 return HAL_FLASH_GetError ();
	     }
	   }

	   /* Lock the Flash to disable the flash control register access (recommended
	      to protect the FLASH memory against possible unwanted operation) *********/
	   HAL_FLASH_Lock();
	   return 0;
}

void Flash_Write_NUM (uint32_t StartSectorAddress, float Num)
{
	uint8_t bytes_temp[4] = {0};
	float2Bytes(bytes_temp, Num);

	FlashWriteData (StartSectorAddress, (uint32_t *)bytes_temp, 1);
}


float Flash_Read_NUM (uint32_t StartSectorAddress)
{
	uint8_t buffer[4];
	float value;

	FlashReadData(StartSectorAddress, (uint32_t *)buffer, 1);
	value = Bytes2float(buffer);
	return value;
}


uint32_t Flash_Write_CalTable (uint32_t StartSectorAddress, gyro_data_t *data)
{
	uint32_t res;
	float temp[3] = {0.0f};
	temp[0] = data->gyro_x;
	temp[1] = data->gyro_y;
	temp[2] = data->gyro_z;
	uint8_t bytes_temp[12] = {0};
	union {
	  float a;
	  uint8_t bytes[4];
	} thing;

	uint8_t j,v=0;
	for (uint8_t i = 0; i < 3; i++){
		thing.a = temp[i];

		for (j = 0; j < 4; j++) {
		  bytes_temp[v+j] = thing.bytes[j];
		}
		v+=4;
	}
	res = FlashWriteData (StartSectorAddress, (uint64_t *)bytes_temp, 3);
	return res;
}

void Flash_Read_CalTable (uint32_t StartSectorAddress, gyro_data_t *data)
{
	uint8_t buffer[20] = {0};
	float temp[3] = {0.0f};

	FlashReadData(StartSectorAddress, (uint64_t *)buffer, 2);
	union {
	  float a;
	  uint8_t bytes[4];
	} thing;

	uint8_t v=0;
	for (uint8_t j = 0; j < 3; j++){
		for (uint8_t i = 0; i < 4; i++) {
			thing.bytes[i] = buffer[v+i];
		}
		v+=4;
		temp[j] =  thing.a;
	}
	data->gyro_x = temp[0];
	data->gyro_y = temp[1];
	data->gyro_z = temp[2];
}

void Convert_To_Str (uint32_t *Data, char *Buf)
{
	int numberofbytes = ((strlen((char *)Data)/4) + ((strlen((char *)Data) % 4) != 0)) *4;

	for (int i=0; i<numberofbytes; i++)
	{
		Buf[i] = Data[i/4]>>(8*(i%4));
	}
}


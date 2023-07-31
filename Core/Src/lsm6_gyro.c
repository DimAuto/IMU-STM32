/*
 * lsm6_gyro.c
 *
 *  Created on: May 25, 2023
 *      Author: dkalaitzakis
 */


#include <string.h>
#include "stm32l4xx_hal.h"
#include "lsm6_gyro.h"
#include "helpers.h"
#include "uart.h"

// I2C object
I2C_HandleTypeDef hi2c2;

static void debugPrintMEMS(mems_data_t *mems_data);

uint16_t gyro_offset_counter = 0;
gyro_data_t gyro_sum;
gyro_data_t gyro_mean;

void tick_gyro(mems_data_t * mems_data){
    gyro_read(mems_data);
    lsm6_acc_read(mems_data);
    lis3_magn_read(mems_data);
//    osDelay(5);
//    debugPrintMEMS(mems_data);
}


uint8_t lsm6_bus_init(void)
{

  hi2c2.Instance = I2C2;
//hi2c2.Init.Timing = 0x00B03FDB;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    return 1;
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    return 2;
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    return 3;
  }
  return 0;

}

uint8_t whoIam_lsm6(void){
	uint8_t addr = 0;
	HAL_StatusTypeDef res = 0;
	res = HAL_I2C_Mem_Read(&hi2c2, LSM6, WHO_AM_I, 1, &addr, 1, 10);
	if (res != HAL_OK){
		return res;
	}
	return addr;
}

uint8_t whoIam_lis3(void){
	uint8_t addr = 0;
	HAL_StatusTypeDef res = 0;
	res = HAL_I2C_Mem_Read(&hi2c2, LIS3_MAGN, WHO_AM_I_MG, 1, &addr, 1, 10);
	if (res != HAL_OK){
		return res;
	}
	return addr;
}

HAL_StatusTypeDef gyro_init(void){
    uint8_t ctrl2_val = 0x50;   //gyro 208Hz-250dps
    uint8_t ctrl3_val = 0x44;   // block data update - reg addr auto incr
    HAL_I2C_Mem_Write(&hi2c2, LSM6, CTRL2_G, I2C_MEMADD_SIZE_8BIT, &ctrl2_val, 1, 20);
    return HAL_I2C_Mem_Write(&hi2c2, LSM6, CTRL3_C, I2C_MEMADD_SIZE_8BIT, &ctrl3_val, 1, 20);
}

HAL_StatusTypeDef lsm6_acc_init(void){
    uint8_t ctrl1_val = 0x50;   //acc off
    uint8_t ctrl10_val = 0x20; //Enable timestamp
    HAL_I2C_Mem_Write(&hi2c2, LSM6, CTRL1_XL, I2C_MEMADD_SIZE_8BIT, &ctrl1_val, 1, 20);
    return HAL_I2C_Mem_Write(&hi2c2, LSM6, CTRL10_C, I2C_MEMADD_SIZE_8BIT, &ctrl10_val, 1, 20);
}

HAL_StatusTypeDef magn_init(void){
	HAL_StatusTypeDef res = HAL_OK;
	uint8_t ctrl1_val = 0x42;
    uint8_t ctrl3_val = 0x00;
    uint8_t ctrl4_val = 0x08;
    uint8_t ctrl5_val = 0x40;
    res = HAL_I2C_Mem_Write(&hi2c2, LIS3_MAGN, CTRL_REG1_MG, I2C_MEMADD_SIZE_8BIT, &ctrl1_val, 1, 50);
    if (res != HAL_OK)return res;
    res = HAL_I2C_Mem_Write(&hi2c2, LIS3_MAGN, CTRL_REG3_MG, I2C_MEMADD_SIZE_8BIT, &ctrl3_val, 1, 50);
    if (res != HAL_OK)return res;
    res = HAL_I2C_Mem_Write(&hi2c2, LIS3_MAGN, CTRL_REG4_MG, I2C_MEMADD_SIZE_8BIT, &ctrl4_val, 1, 50);
    if (res != HAL_OK)return res;
    res = HAL_I2C_Mem_Write(&hi2c2, LIS3_MAGN, CTRL_REG5_MG, I2C_MEMADD_SIZE_8BIT, &ctrl5_val, 1, 50);
    return res;
}

HAL_StatusTypeDef gyro_read(mems_data_t *mems_data){
	uint8_t data[6]={0};
	uint8_t ts_data[4]={0};
	int16_t gyro_x, gyro_y, gyro_z;
	HAL_StatusTypeDef res = HAL_OK;
    HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTX_L_G, I2C_MEMADD_SIZE_8BIT, data, 6, 50);
    if (res != HAL_OK){
		return res;
	}
    HAL_I2C_Mem_Read(&hi2c2, LSM6, TIMESTAMP0, I2C_MEMADD_SIZE_8BIT, ts_data, 4, 50);
    if (res != HAL_OK){
		return res;
	}
    gyro_x = ((int16_t)((data[1] << 8) | data[0]));
    gyro_y = ((int16_t)((data[3] << 8) | data[2]));
    gyro_z = ((int16_t)((data[5] << 8) | data[4]));
    mems_data->gyro.gyro_x = (gyro_x / -131.1f);// * -1.0f;
    mems_data->gyro.gyro_y = (gyro_y / -131.1f);// * -1.0f;
    mems_data->gyro.gyro_z = (gyro_z / 131.1f);// * -1.0f;
    mems_data->timestamp = ((int)((ts_data[3]<<24)|(ts_data[2]<<16)|(ts_data[1]<<8)|(ts_data[0])));
    return res;
}

HAL_StatusTypeDef lsm6_acc_read(mems_data_t *mems_data){
	uint8_t data[6] = {0};
	int16_t acc_x, acc_y, acc_z;
	HAL_StatusTypeDef res = HAL_OK;
	res = HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTX_L_A, I2C_MEMADD_SIZE_8BIT, data, 6, 50);
	if (res != HAL_OK){
		return res;
	}
    acc_x = ((int16_t)((data[1] << 8) | data[0]));
    acc_y = ((int16_t)((data[3] << 8) | data[2]));
    acc_z = ((int16_t)((data[5] << 8) | data[4]));
    mems_data->acc.acc_x = (acc_x / -16384.0f);//  * -1.0f;
    mems_data->acc.acc_y = (acc_y / -16384.0f);// * -1.0f;
    mems_data->acc.acc_z = (acc_z / 16384.0f);// * -1.0f;
    return res;
}

HAL_StatusTypeDef lis3_magn_read(mems_data_t *mems_data){
	uint8_t data[6] = {0};
    int16_t magn_x, magn_y, magn_z;
    HAL_StatusTypeDef res = HAL_OK;

    HAL_I2C_Mem_Read(&hi2c2, LIS3_MAGN, OUT_X_L_MG, I2C_MEMADD_SIZE_8BIT, data, 6, 50);
    if (res != HAL_OK){
    	return res;
	}
    magn_x = ((int16_t)((data[1] << 8) | data[0]));
    magn_y = ((int16_t)((data[3] << 8) | data[2]));
    magn_z = ((int16_t)((data[5] << 8) | data[4]));
    mems_data->magn.magn_x = magn_x / 10.0f;
    mems_data->magn.magn_y = magn_y / 10.0f;
    mems_data->magn.magn_z = magn_z / 10.0f;
    return res;
}


uint8_t gyro_offset_calculation(mems_data_t *mems_data){
	gyro_read(mems_data);
	gyro_sum.gyro_x += mems_data->gyro.gyro_x;
	gyro_sum.gyro_y += mems_data->gyro.gyro_y;
	gyro_sum.gyro_z += mems_data->gyro.gyro_z;
	gyro_offset_counter++;
	if (gyro_offset_counter >= 1400){
		gyro_mean.gyro_x = gyro_sum.gyro_x / gyro_offset_counter;
		gyro_mean.gyro_y = gyro_sum.gyro_y / gyro_offset_counter;
		gyro_mean.gyro_z = gyro_sum.gyro_z / gyro_offset_counter;
		setGyroOffset(gyro_mean);
		gyro_offset_counter = 0;
		return 0;
	}
	return 1;
}

void debugPrintMEMS(mems_data_t *mems_data){
	uint8_t text[20] = {0};
//	uart_write("Raw:", 0, UART_NYX, 50);
//	memcpy(text,0,20);
//	sprintf(text, "%d\r\n,", mems_data->timestamp);
//	uart_write_debug(text, 50);
//	memcpy(text,0,20);
//	sprintf(text, "%d,", mems_data->acc_x);

//	memcpy(text,0,20);
//	sprintf(text, "%d,", mems_data->acc_y);
//	uart_write(text, 0, UART_NYX, 50);
//	memcpy(text,0,20);
//	sprintf(text, "%d,", mems_data->acc_z);
//	uart_write(text, 0, UART_NYX, 50);
//	memcpy(text,0,20);
	sprintf(text, "%f,", mems_data->gyro.gyro_x);
	uart_write_debug(text, 20);
	memcpy(text,0,20);
	sprintf(text, "%f,", mems_data->gyro.gyro_y);
	uart_write_debug(text, 20);
	memcpy(text,0,20);
	sprintf(text, "%f\r\n", mems_data->gyro.gyro_z);
	uart_write_debug(text, 20);
	memcpy(text,0,20);
//	sprintf(text, "%f,", mems_data->magn_x);
//	uart_write_debug(text, 20);
//	memcpy(text,0,20);
//	sprintf(text, "%d,", mems_data->magn_y);
//	uart_write(text, 0, UART_NYX, 50);
//	memcpy(text,0,20);
//	sprintf(text, "%d\r\n", mems_data->magn_z);
//	uart_write(text, 0, UART_NYX, 50);
//	memcpy(text,0,20);
}



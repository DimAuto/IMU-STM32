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


void tick_gyro(mems_data_t * mems_data){
    gyro_read(mems_data);
    lsm6_acc_read(mems_data);
    lis3_magn_read(mems_data);
    osDelay(5);
    debugPrintMEMS(mems_data);
}


uint8_t lsm6_bus_init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00B03FDB;
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
    res = HAL_I2C_Mem_Write(&hi2c2, LIS3_MAGN, CTRL_REG1_MG, I2C_MEMADD_SIZE_8BIT, &ctrl1_val, 1, 20);
    if (res != HAL_OK)return res;
    res = HAL_I2C_Mem_Write(&hi2c2, LIS3_MAGN, CTRL_REG3_MG, I2C_MEMADD_SIZE_8BIT, &ctrl3_val, 1, 20);
    if (res != HAL_OK)return res;
    res = HAL_I2C_Mem_Write(&hi2c2, LIS3_MAGN, CTRL_REG4_MG, I2C_MEMADD_SIZE_8BIT, &ctrl4_val, 1, 20);
    if (res != HAL_OK)return res;
    res = HAL_I2C_Mem_Write(&hi2c2, LIS3_MAGN, CTRL_REG5_MG, I2C_MEMADD_SIZE_8BIT, &ctrl5_val, 1, 20);
    return res;
}

HAL_StatusTypeDef gyro_read(mems_data_t *mems_data){
	uint8_t x_l, x_h = 0;
	uint8_t y_l, y_h = 0;
	uint8_t z_l, z_h = 0;
	uint8_t tm0,tm1,tm2,tm3;
	int16_t gyro_x, gyro_y, gyro_z;
	HAL_StatusTypeDef res = HAL_OK;
    HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTX_L_G, 1, &x_l, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTX_H_G, 1, &x_h, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTY_L_G, 1, &y_l, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTY_H_G, 1, &y_h, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTZ_L_G, 1, &z_l, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTZ_H_G, 1, &z_h, 1, 30);
    /*Timestamp Read*/
    HAL_I2C_Mem_Read(&hi2c2, LSM6, TIMESTAMP0, 1, &tm0, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LSM6, TIMESTAMP1, 1, &tm1, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LSM6, TIMESTAMP2, 1, &tm2, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LSM6, TIMESTAMP3, 1, &tm3, 1, 30);

    HAL_I2C_Mem_Read(&hi2c2, LSM6, TIMESTAMP3, 1, &tm3, 1, 30);


    gyro_x = ((int16_t)((x_h << 8) | x_l));
    gyro_y = ((int16_t)((y_h << 8) | y_l));
    gyro_z = ((int16_t)((z_h << 8) | z_l));
    mems_data->gyro_x = (gyro_x / 131.1);// * -1.0f;
    mems_data->gyro_y = (gyro_y / 131.1);// * -1.0f;
    mems_data->gyro_z = (gyro_z / 131.1) * -1.0f;
    mems_data->timestamp = ((int)((tm3<<24)|(tm2<<16)|(tm1<<8)|(tm0)));
    return res;
}

HAL_StatusTypeDef lsm6_acc_read(mems_data_t *mems_data){
	uint8_t x_l, x_h = 0;
	uint8_t y_l, y_h = 0;
	uint8_t z_l, z_h = 0;
	int16_t acc_x, acc_y, acc_z;
	HAL_StatusTypeDef res = HAL_OK;
	HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTX_L_A, 1, &x_l, 1, 30);
	HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTX_H_A, 1, &x_h, 1, 30);
	HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTY_L_A, 1, &y_l, 1, 30);
	HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTY_H_A, 1, &y_h, 1, 30);
	HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTZ_L_A, 1, &z_l, 1, 30);
	HAL_I2C_Mem_Read(&hi2c2, LSM6, OUTZ_H_A, 1, &z_h, 1, 30);
    acc_x = ((int16_t)((x_h << 8) | x_l));
    acc_y = ((int16_t)((y_h << 8) | y_l));
    acc_z = ((int16_t)((z_h << 8) | z_l));
    mems_data->acc_x = (acc_x / 16384.0f);//  * -1.0f;
    mems_data->acc_y = (acc_y / 16384.0f);// * -1.0f;
    mems_data->acc_z = (acc_z / 16384.0f) * -1.0f;
    return res;
}

HAL_StatusTypeDef lis3_magn_read(mems_data_t *mems_data){
    uint8_t x_l, x_h = 0;
    uint8_t y_l, y_h = 0;
    uint8_t z_l, z_h = 0;
    int16_t magn_x, magn_y, magn_z;
    HAL_StatusTypeDef res = HAL_OK;

    HAL_I2C_Mem_Read(&hi2c2, LIS3_MAGN, OUT_X_L_MG, 1, &x_l, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LIS3_MAGN, OUT_X_H_MG, 1, &x_h, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LIS3_MAGN, OUT_Y_L_MG, 1, &y_l, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LIS3_MAGN, OUT_Y_H_MG, 1, &y_h, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LIS3_MAGN, OUT_Z_L_MG, 1, &z_l, 1, 30);
    HAL_I2C_Mem_Read(&hi2c2, LIS3_MAGN, OUT_Z_H_MG, 1, &z_h, 1, 30);
    magn_x = ((int16_t)((x_h << 8) | x_l));
    magn_y = ((int16_t)((y_h << 8) | y_l));
    magn_z = ((int16_t)((z_h << 8) | z_l));
    mems_data->magn_x = magn_x / 10.0f;
    mems_data->magn_y = magn_y / 10.0f;
    mems_data->magn_z = magn_z / 10.0f;
    return res;
}

void debugPrintMEMS(mems_data_t *mems_data){
	uint8_t text[20] = {0};
//	uart_write_debug("Raw:", 50);
//	memcpy(text,0,10);
//	sprintf(text, "%f,", mems_data->acc_x);
//	uart_write_debug(text, 50);
//	memcpy(text,0,10);
//	sprintf(text, "%f,", mems_data->acc_y);
//	uart_write_debug(text, 50);
//	memcpy(text,0,10);
//	sprintf(text, "%f,", mems_data->acc_z);
//	uart_write_debug(text, 50);
//	memcpy(text,0,10);
//	sprintf(text, "%f,", mems_data->gyro_x);
//	uart_write_debug(text, 50);
//	memcpy(text,0,10);
//	sprintf(text, "%f,", mems_data->gyro_y);
//	uart_write_debug(text, 50);
//	memcpy(text,0,10);
//	sprintf(text, "%f,", mems_data->gyro_z);
//	uart_write_debug(text, 50);
//	memcpy(text,0,10);
	sprintf(text, "%f,", mems_data->magn_x);
	uart_write_uart4(text, 50);
	memcpy(text,0,10);
	sprintf(text, "%f,", mems_data->magn_y);
	uart_write_uart4(text, 50);
	memcpy(text,0,10);
	sprintf(text, "%f\r\n", mems_data->magn_z);
	uart_write_uart4(text, 50);
	memcpy(text,0,10);
}



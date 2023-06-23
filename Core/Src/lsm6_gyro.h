/*
 * lsm6_gyro.h
 *
 *  Created on: May 25, 2023
 *      Author: dkalaitzakis
 */

#ifndef SRC_LSM6_GYRO_H_
#define SRC_LSM6_GYRO_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

#define LSM6		(0x6A << 1)
#define LIS3_MAGN	(0x1E << 1)

#define INT1_CTRL   0x0D
#define INT2_CTRL   0x0E
#define WHO_AM_I    0x0F
#define CTRL1_XL    0x10
#define CTRL2_G     0x11
#define CTRL3_C     0x12
#define CTRL4_C     0x13
#define CTRL5_C     0x14
#define CTRL7_G     0x16
#define	CTRL10_C    0x19
#define STATUS_REG  0x1E
#define OUTX_L_G    0x22
#define OUTX_H_G    0x23
#define OUTY_L_G    0x24
#define OUTY_H_G    0x25
#define OUTZ_L_G    0x26
#define OUTZ_H_G    0x27
#define OUTX_L_A    0x28
#define OUTX_H_A    0x29
#define OUTY_L_A    0x2A
#define OUTY_H_A    0x2B
#define OUTZ_L_A    0x2C
#define OUTZ_H_A    0x2D
#define EMB_FUNC_STATUS_MAINPAGE    0x35
#define INTERNAL_FREQ_FINE          0x63
#define TIMESTAMP0	0x40
#define TIMESTAMP1	0x41
#define TIMESTAMP2	0x42
#define TIMESTAMP3	0x43

#define WHO_AM_I_MG			0x0F
#define CTRL_REG1_MG		0x20
#define CTRL_REG2_MG		0x21
#define CTRL_REG3_MG		0x22
#define CTRL_REG4_MG		0x23
#define CTRL_REG5_MG		0x24
#define STATUS_REG_MG		0x27
#define OUT_X_L_MG			0x28
#define OUT_X_H_MG			0x29
#define OUT_Y_L_MG			0x2A
#define OUT_Y_H_MG			0x2B
#define OUT_Z_L_MG			0x2C
#define OUT_Z_H_MG			0x2D



typedef struct{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float magn_x;
    float magn_y;
    float magn_z;
    int	timestamp;
}mems_data_t;

uint8_t whoIam_lsm6(void);

uint8_t whoIam_lis3(void);

uint8_t lsm6_bus_init(void);

HAL_StatusTypeDef gyro_init(void);

HAL_StatusTypeDef magn_init(void);

HAL_StatusTypeDef gyro_read(mems_data_t *mems_data);

HAL_StatusTypeDef lsm6_acc_init(void);

HAL_StatusTypeDef lsm6_acc_read(mems_data_t *mems_data);

HAL_StatusTypeDef lis3_magn_read(mems_data_t *mems_data);

void tick_gyro(mems_data_t *mems_data);

#endif /* SRC_LSM6_GYRO_H_ */

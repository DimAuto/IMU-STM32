/*
 * accel_lsm303.c
 *
 *  Created on: Feb 23, 2023
 *      Author: dkalaitzakis
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <src/lsm303.h>
#include <src/config.h>
#include "core/i2c.h"
#include "core/uart.h"
#include "core/helpers.h"
#include "message_handler.h"

static acc_data_t acc_data;
static magn_data_t mgn_data;
static iitFilter_t iitFilter;

void tick_accelerometer(void){
    accelerometer_read();
}

void tick_magnetometer(void){
    magnetometer_read();
}

void accel_transmit_message(uint8_t cmd, UART_select device){
    uint8_t message[6] = {0};
    message[0] = (acc_data.acc_x >> 8);
    message[1] = acc_data.acc_x & 0xFF;
    message[2] = (acc_data.acc_y >> 8);
    message[3] = acc_data.acc_y & 0xFF;
    message[4] = (acc_data.acc_z >> 8);
    message[5] = acc_data.acc_z & 0xFF;

    transmitMessage(message, 6, cmd, device);
}

void magn_transmit_message(uint8_t cmd, UART_select device){
    uint8_t message[6] = {0};
    message[0] = (mgn_data.magn_x >> 8);
    message[1] = mgn_data.magn_x & 0xFF;
    message[2] = (mgn_data.magn_y >> 8);
    message[3] = mgn_data.magn_y & 0xFF;
    message[4] = (mgn_data.magn_z >> 8);
    message[5] = mgn_data.magn_z & 0xFF;

    transmitMessage(message, 6, cmd, device);
}

//uint8_t accelerometer_init(acc_power_modes mode){
//    uint8_t cfg_reg1 = 0;
//    uint8_t cfg_reg4 = 0;
//    uint8_t res = 0;
//    acc_data.acc_x = 0;
//    acc_data.acc_y = 0;
//    acc_data.acc_z = 0;
//    acc_data.mode = mode;
//
//    switch(mode){
//    case LOW_POWER_MODE:
//        cfg_reg1 = 0x7F;
//        cfg_reg4 = 0x00;  //Setup for 400Hz low power mode
//        break;
//    case NORMAL_MODE:
//        cfg_reg1 = 0x77;
//        cfg_reg4 = 0x00;
//        break;
//    case HIGH_RES_MODE:
//        cfg_reg1 = 0x77;
//        cfg_reg4 = 0x08;  //Setup for 400Hz High res mode
//        break;
//    }
//    res = I2C_Master_WriteReg(LSM303_ACC, CTRL_REG1_A, &cfg_reg1, 1, I2C_SEL_ACC);
//    res = I2C_Master_WriteReg(LSM303_ACC, CTRL_REG4_A, &cfg_reg4, 1, I2C_SEL_ACC);
////#ifdef __DEBUG__
//    if (res!=8){
//        uart_write_DEBUG("Accelerometer Initialized ",UART_NYX);
//        uart_write_DEBUG("\n",UART_NYX);
//    }
//    else{
//        uart_write_DEBUG("Accelerometer Initialization Failed! ",UART_NYX);
//        uart_write_DEBUG("\n",UART_NYX);
//    }
////#endif
//    return res;
//}


uint8_t magnetometer_init(uint8_t mode){
    uint8_t cfg_regA = 0x8C;
    uint8_t cfg_regB = 0x01;
    uint8_t res = 0;
    mgn_data.magn_offset_x = 0;
    mgn_data.magn_offset_y = 0;
    mgn_data.magn_offset_z = 0;
    mgn_data.magn_x = 0;
    mgn_data.magn_y = 0;
    mgn_data.magn_z = 0;
    memset(iitFilter.valX, 0, 3);
    memset(iitFilter.valY, 0, 3);
    memset(iitFilter.valZ, 0, 3);
    memset(iitFilter.fiitX, 0, 3);
    memset(iitFilter.fiitY, 0, 3);
    memset(iitFilter.fiitZ, 0, 3);
    res = I2C_Master_WriteReg(LSM303_MGN, CFG_REG_A_M, &cfg_regA, 1, I2C_SEL_ACC);
    res = I2C_Master_WriteReg(LSM303_MGN, CFG_REG_B_M, &cfg_regB, 1, I2C_SEL_ACC);
//#ifdef __DEBUG__
    if (res!=8){
        uart_write_DEBUG("Magnetometer Initialized ",UART_NYX);
        uart_write_DEBUG("\n",UART_NYX);
    }
    else{
        uart_write_DEBUG("Magnetometer Initialization Failed! ",UART_NYX);
        uart_write_DEBUG("\n",UART_NYX);
    }
//#endif
    return res;

}

uint8_t lsm303_check_acc(void){
    uint8_t data = 0;
    uint8_t res = 0;

    res = I2C_Master_ReadReg(LSM303_ACC, WHO_AM_I_A, 1, &data, I2C_SEL_ACC);
    if ((res==8)||(data != 51)) return 0; //If the i2c gets timed out or the returned data is different than the acc's address.
    return data;
}

uint8_t lsm303_check_mgn(void){
    uint8_t data = 0;
    uint8_t res = 0;

    res = I2C_Master_ReadReg(LSM303_MGN, WHO_AM_I_M, 1, &data, I2C_SEL_ACC);
    if ((res==8)||(data != 64)) return 0; //If the i2c gets timed out or the returned data is different than the mgn's address.
    return data;
}

uint8_t magnetometer_read(void){
//    uint8_t x_l, x_h, y_l, y_h, z_l, z_h;
    uint8_t mag_data[6]={0};

    uint8_t OUT_AUTO_I_M = (OUTX_L_REG_M | OUT_AUTO_I);

    if(I2C_Master_ReadReg(LSM303_MGN, OUT_AUTO_I_M, 6, mag_data, I2C_SEL_ACC) == 8)return 8;
    mgn_data.magn_x = (int16_t)((mag_data[1] << 8) | mag_data[0]);
    mgn_data.magn_y = (int16_t)((mag_data[3] << 8) | mag_data[2]);
    mgn_data.magn_z = (int16_t)((mag_data[5] << 8) | mag_data[4]);

//#ifdef __DEBUG__
    uint8_t mes[14]={0};
    itoa(mgn_data.magn_x, mes,10);
//    uart_write_DEBUG("MGN\r\n",UART_NYX);
//    delayMS(1);
    uart_write_DEBUG(mes,UART_NYX);
    memset(mes,0,14);
    itoa(mgn_data.magn_y, mes,10);
    uart_write_DEBUG(",",UART_NYX);
    delayMS(1);
    uart_write_DEBUG(mes,UART_NYX);
    memset(mes,0,14);
    itoa(mgn_data.magn_z, mes,10);
    uart_write_DEBUG(",",UART_NYX);
    delayMS(1);
    uart_write_DEBUG(mes,UART_NYX);
    memset(mes,0,14);
    delayMS(1);
    uart_write_DEBUG("\r\n",UART_NYX);
//#endif
    return 0;
}

//uint8_t magnetometer_offset_read(void){
//    uint8_t x_l, x_h, y_l, y_h, z_l, z_h;
//    uint8_t dta = 0;
//    if(I2C_Master_ReadReg(LSM303_MGN, OFFSET_X_REG_L_M, 1, &x_l, I2C_SEL_ACC) == 8)return 8;
//    if(I2C_Master_ReadReg(LSM303_MGN, OFFSET_X_REG_H_M, 1, &x_h, I2C_SEL_ACC) == 8)return 8;
//    mgn_data.magn_offset_x = (int16_t)((x_h << 8) | x_l);
//    if(I2C_Master_ReadReg(LSM303_MGN, OFFSET_Y_REG_L_M, 1, &y_l, I2C_SEL_ACC) == 8)return 8;
//    if(I2C_Master_ReadReg(LSM303_MGN, OFFSET_Y_REG_H_M, 1, &y_h, I2C_SEL_ACC) == 8)return 8;
//    mgn_data.magn_offset_y = (int16_t)((y_h << 8) | y_l);
//    if(I2C_Master_ReadReg(LSM303_MGN, OFFSET_Z_REG_L_M, 1, &z_l, I2C_SEL_ACC) == 8)return 8;
//    if(I2C_Master_ReadReg(LSM303_MGN, OFFSET_Z_REG_H_M, 1, &z_h, I2C_SEL_ACC) == 8)return 8;
//    mgn_data.magn_offset_z = (int16_t)((z_h << 8) | z_l);
//    return 0;
//}

//uint16_t accelerometer_read(void){
//    uint8_t data[6]={0};
//    uint8_t dta = 0;
//    uint8_t OUT_AUTO_I_A = (OUT_X_L_A | OUT_AUTO_I);
//
////    while(!dta){
////        I2C_Master_ReadReg(LSM303_ACC, STATUS_REG_A, 1, &dta, I2C_SEL_ACC);
////    }
//    if(I2C_Master_ReadReg(LSM303_ACC, OUT_AUTO_I_A, 6, data, I2C_SEL_ACC) == 8)return 8;
//    acc_data.acc_x = ((int16_t)((data[1] << 8) | data[0])) >> acc_data.mode; //to get the 12bit of HR.
//    acc_data.acc_y = ((int16_t)((data[3] << 8) | data[2])) >> acc_data.mode;
//    acc_data.acc_z = ((int16_t)((data[5] << 8) | data[4])) >> acc_data.mode;
////    data->g_x = data->acc_x * HR_LSB;
////    data->g_y = data->acc_y * HR_LSB;
////    data->g_z = data->acc_z * HR_LSB;
////#ifdef __DEBUG__
//    uint8_t mes[10]={0};
//    itoa(acc_data.acc_x, mes,10);
//    uart_write_DEBUG(",",UART_NYX);
//    delayMS(1);
//    uart_write_DEBUG(mes,UART_NYX);
//    memset(mes,0,10);
//    itoa(acc_data.acc_y, mes,10);
//    uart_write_DEBUG(",",UART_NYX);
//    delayMS(1);
//    uart_write_DEBUG(mes,UART_NYX);
//    memset(mes,0,10);
//    itoa(acc_data.acc_z, mes,10);
//    uart_write_DEBUG(",",UART_NYX);
//    delayMS(1);
//    uart_write_DEBUG(mes,UART_NYX);
//    memset(mes,0,10);
////#endif
//    return 0;
//}


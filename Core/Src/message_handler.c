/*
 * bus.c
 *
 *  Created on: Feb 25, 2023
 *      Author: kalai
 */

#include <src/message_handler.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "core/uart.h"
#include "core/helpers.h"
#include "core/i2c.h"
#include "core/adc.h"
#include "config.h"
#include "gps_neoM9N.h"
#include "lsm303.h"
#include "core/timer.h"
#include "battery_monitor.h"
#include "sbp_gpio.h"

// Global variables
extern uint8_t transEnd_debug;
extern uint8_t transEnd_iris;
extern uint8_t transEnd_nyx;
//-----------------------

static uint8_t TOKEN = 0;


static void calcChecksum(void);
static uint8_t calcDataSize(uint8_t *data, uint8_t len);
static void jump_BSL(void);
static void init_message_t(void);
static void handler(UART_select device);

uint8_t flag_connected_toIris = 0;

static uint8_t message_d[25]={0};
static message_t msg;

void tick_Handler(void){
    uint16_t mess_len = 0;
    init_message_t();

    if (transEnd_iris){
        mess_len = uart_RX_getSize(UART_IRIS);
        uart_read_RB(message_d, UART_IRIS);
        if (!parseMessage(message_d, mess_len, UART_IRIS)){
            handler(UART_IRIS);
        }
    }

    if (transEnd_debug){
        mess_len = uart_RX_getSize(UART_DEBUG);
        uart_read_RB(message_d, UART_DEBUG);
        if (!parseMessage(message_d, mess_len, UART_DEBUG)){
            handler(UART_DEBUG);
        }
    }

    if (transEnd_nyx){
        mess_len = uart_RX_getSize(UART_NYX);
        uart_read_RB(message_d, UART_NYX);
        if (!parseMessage(message_d, mess_len, UART_NYX)){
            handler(UART_NYX);
        }
    }
}

uint8_t send_heartbeat(UART_select device){
    uint8_t data[1] = {0};
    return transmitMessage(data, 1, SBP_CMD_HEARTBEAT, device);
}

uint8_t sendNack(UART_select device){
    if ((TOKEN == ESC) || (TOKEN == ETX) || (TOKEN == STX)){
        uint8_t msg[3] = {NACK,ESC,TOKEN};
        return uart_write_RB(msg, 3, device);
    }
    else{
        uint8_t msg[2] = {NACK,TOKEN};
        return uart_write_RB(msg, 2, device);
    }
}

uint8_t sendAck(UART_select device){
    if ((TOKEN == ESC) || (TOKEN == ETX) || (TOKEN == STX)){
        uint8_t msg[3] = {ACK,ESC,TOKEN};
        return uart_write_RB(msg, 3, device);
    }
    else{
        uint8_t msg[2] = {ACK,TOKEN};
        return uart_write_RB(msg, 2, device);
    }
}

//uint16_t get_heartbeat_timeout(void){
//    return hb_timeout;
//}

static uint8_t calcDataSize(uint8_t *data, uint8_t len){
    uint8_t j,i;
    j=0;
    for (i=0;i<len;i++){
       if ((data[i] == STX) || (data[i] == ETX) || (data[i] == ESC)){
           j++;
           j++;
       }
       else j++;
    }
    return j;
}

uint8_t transmitMessage(uint8_t *data, uint8_t data_len, uint8_t cmd, UART_select device){
    uint8_t message[34];
    uint8_t i,j,index;
    uint8_t tmp_len = 0;

    tmp_len = calcDataSize(data, data_len);
    msg.len = data_len;
//    message = malloc((tmp_len+14)*sizeof(uint8_t));
//    if (!message)return 1;
//    memset(message, 0,tmp_len+14);
    memcpy(msg.data, data, msg.len);
    msg.protocol_rev[0] = PROTOCOL_VER;
    msg.protocol_rev[1] = COMM_PROTOCOL_REV;
    if (TOKEN == 255){
        TOKEN=0;
    }
    else{
        TOKEN++;
    }
    msg.token = TOKEN;

    msg.cmd = cmd;
    msg.senderID[0] = ESC_CHAR;
    msg.senderID[1] = SBP_S_ID;
//    msg.cmd = msg.cmd; Remains the same
    calcChecksum();
    //CREATE MESSAGE
    index = 0;
    message[index] = STX;
    index++;
    message[index] = msg.protocol_rev[0];
    index++;
    message[index] = msg.protocol_rev[1];
    index++;
    if ((msg.token == STX) || (msg.token == ETX) || (msg.token == ESC)){
        message[index] = ESC;
        index++;
        message[index] = msg.token;
        index++;
    }
    else{
        message[index] = msg.token;
        index++;
    }
    message[index] = msg.senderID[0];
    index++;
    message[index] = msg.senderID[1];
    index++;
    message[index] = msg.cmd;
    index++;
    for(i=0; i<msg.len;i++){
        if ((msg.data[i] == STX) || (msg.data[i] == ETX) || (msg.data[i] == ESC)){
            message[index] = ESC;
            index++;
            message[index] = msg.data[i];
            index++;
        }
        else{
            message[index] = msg.data[i];
            index++;
        }
    }
    message[index] = msg.checksum[3];
    index++;
    message[index] = msg.checksum[2];
    index++;
    message[index] = msg.checksum[1];
    index++;
    if (msg.checksum[0] == ESC){
        message[index] = ESC;
        index++;
        message[index] = msg.checksum[0];
        index++;
        message[index] = ETX;
        index++;
        msg.len = index;
    }
    else{
        message[index] = msg.checksum[0];
        index++;
        message[index] = ETX;
        index++;
        msg.len = index;
    }
    uart_write_RB(message, msg.len , device);
    uint16_t timeout = millis();
    while((ellapsed_millis(timeout) < 150)){
        if (uart_wait_ACK(device) == 0){
            return 0;
        }
        else{
            return 1;
        }
    }
    return 1;
}

static void calcChecksum(void){
    msg.checksum[0] = msg.protocol_rev[0];
    msg.checksum[0] ^= msg.protocol_rev[1];

    msg.checksum[0] ^= msg.token;

    msg.checksum[0] ^= msg.senderID[0];
    msg.checksum[0] ^= msg.senderID[1];

    msg.checksum[0] ^= msg.cmd;
    uint8_t i;

    for (i=0; i < msg.len; i++)
    {
       msg.checksum[0] ^= msg.data[i];
    }
}


uint8_t parseMessage(uint8_t *data, uint8_t len, UART_select device){
    uint8_t chsum = 0;
    if (len < 11){
        sendNack(device);
        return 1;
    }
    msg.len = len-11;
    if (!msg.data)return 2;
    msg.protocol_rev[0] = data[0];
    msg.protocol_rev[1] = data[1];
    msg.token = data[2];
    TOKEN = msg.token;
    msg.senderID[0] = data[3];
    msg.senderID[1] = data[4];
    msg.cmd = data[5];
    uint8_t i;
    for (i=0; i<msg.len; i++){
//        if (data[6+i] == ESC) i++;
        msg.data[i] = data[6+i];
    }
    msg.checksum[3] = data[6+i];
    msg.checksum[2] = data[7+i];
    msg.checksum[1] = data[8+i];
    msg.checksum[0] = data[9+i];
    chsum = msg.checksum[0];
    calcChecksum();
    if (chsum != msg.checksum[0]){
        sendNack(device);
//        uart_write_DEBUG("NACK\r\n", UART_NYX);
        return 1;
    }
    sendAck(device);
    delayMS(22);
//    uart_write_DEBUG("ACK\r\n", UART_NYX);
    return 0;                  //Note that after the parsing the escape chars remains in payload.
}



void handler(UART_select device){
    switch (msg.cmd){
    case 0x50:
        flag_connected_toIris = 1;
        break;
    case 0x51:break;
    case 0x56:
        ublox_transmit_message(msg.cmd, device);
        break;
    case 0x57:
        magn_transmit_message(msg.cmd, device);
        break;
    case 0x58:
        accel_transmit_message(msg.cmd, device);
        break;
    case 0x60:
        jump_BSL();
        break;
    case 0x65:
        reportFW(msg.cmd, device);
        break;
    case 0x70:
        transmit_battery_voltage(msg.cmd, device);
        break;
    case 0x72:
        transmit_external_voltage(msg.cmd, device);
        break;
    case 0x74:
        transmit_power_line(msg.cmd, device);
        break;
//    case 0x78:
//        transmit_nyx_consumption(msg.cmd, device);
//        break;
    case 0x80:
        ublox_transmit_rtc(msg.cmd, device);
        break;
    case 0x81:
        transmit_nyx_consumption(msg.cmd, device);
        break;
    case 0x82:
        transmit_nyx_flipped_flag(msg.cmd, device);
        break;
    case 0xA0:
        gpio_setGNSS_RESET(PIN_LOW);
        delayMS(500);
        gpio_setGNSS_RESET(PIN_HIGH);
        break;
    default:
        break;
    }
}

void init_message_t(void){
    msg.protocol_rev[0] = 0;
    msg.protocol_rev[1] = 0;
    msg.token = 0;
    msg.senderID[0] = 0;
    msg.senderID[1] = 0;
    msg.cmd = 0;
    memset(msg.data, 0, 20);
    msg.checksum[0] = 0;
    msg.checksum[1] = 0;
    msg.checksum[2] = 0;
    msg.checksum[3] = 0;
}

void reportFW(uint8_t cmd, UART_select device){
    uint8_t fwv[1] = {FW_VERSION};
    transmitMessage(fwv, 1, cmd, device);
}

void jump_BSL(void){
#ifdef __DEBUG__
    uart_write_DEBUG("ENTERED_BSL",UART_DEBUG);
    uart_write_DEBUG("\n",UART_DEBUG);
#endif
    watchdog_stop();
    MPUCTL0 = MPUPW;    //DISABLE MEMORY PROTECT UNIT !! MANDATORY !!
    delayMS(100);
    _disable_interrupt(); // disable interrupts
    ((void ( * )())0x1000)();   // jump to BSL
}



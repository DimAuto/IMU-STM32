/*
 * gps_neoM9N.c
 *
 *  Created on: Feb 7, 2023
 *      Author: dkalaitzakis
 */

#include "gps_neoM9N.h"
#include <string.h>


static void calcChecksum(messageCFG_t *msg);

//gps_data_t *gps_data;
static messageCFG_t config_message = {UBX_SYNCH_2, 0, 0, 0, 0, 0, 1, 0, 0, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED, SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
static gps_data_t gps_data;
static uint8_t gps_loss_count = 0;

uint8_t gps_data_backup_flag = 0;   //Flag to enable gps data backup only on boot.


void ublox_tick(void){
    uint8_t res = 0;
//    uint8_t *data;
    res = ubloxRead();
//    data = malloc(76 * sizeof(uint8_t));
    if ((res == 8) || (res==10)){
#ifdef __DEBUG__
        uart_write_DEBUG("Failed to read\r\n",UART_NYX);
#endif
        return;
    }
    else{
        parseNMEA();
    }
}

void ublox_transmit_rtc(uint8_t cmd, UART_select device){
    transmitMessage(gps_data.timestamp, 9, cmd, device);
}

void ublox_transmit_message(uint8_t cmd, UART_select device){
    uint8_t message[12] = {0};
    message[0] = (gps_data.latitude & 0xFF000000) >> 24;
    message[1] = (gps_data.latitude & 0x00FF0000) >> 16;
    message[2] = (gps_data.latitude & 0x0000FF00) >> 8;
    message[3] = (gps_data.latitude & 0x000000FF);
    message[4] = (gps_data.longtitude & 0xFF000000) >> 24;
    message[5] = (gps_data.longtitude & 0x00FF0000) >> 16;
    message[6] = (gps_data.longtitude & 0x0000FF00) >> 8;
    message[7] = (gps_data.longtitude & 0x000000FF);
    message[8] = (gps_data.altitude & 0xFF000000) >> 24;
    message[9] = (gps_data.altitude & 0x00FF0000) >> 16;
    message[10] = (gps_data.altitude & 0x0000FF00) >> 8;
    message[11] = (gps_data.altitude & 0x000000FF);
    transmitMessage(message, 12, cmd, device);
}

uint8_t ubloxInit(void){
    setPortOutput(COM_PORT_I2C, COM_TYPE_NMEA);
    configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GLL, NMEA_GGL_RATE, COM_PORT_I2C);
    configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GSA, NMEA_GSA_RATE, COM_PORT_I2C);
    configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GSV, NMEA_GSV_RATE, COM_PORT_I2C);
    configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_RMC, NMEA_RMC_RATE, COM_PORT_I2C);
    configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_VTG, NMEA_VTG_RATE, COM_PORT_I2C);
    configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GGA, NMEA_GGA_RATE, COM_PORT_I2C);
//    return saveConfiguration();
    return 0;
}

void ubloxNmeaGGA_set_refresh_rate(uint8_t seconds){
    configureNMEA(UBX_CLASS_NMEA, UBX_NMEA_GGA, seconds, COM_PORT_I2C);
}

uint8_t ubloxRead(void){
    uint8_t res = 0;
    uint16_t num = 0;
    uint8_t bytes[2] = {0};

    res = I2C_Master_ReadReg(UBLOX_M9N, 0xFD, 2, bytes, I2C_SEL_GPS);
    if (res!=0)return res;
    num  = ((bytes[0] << 8) | bytes[1]);
    memset(bytes, 0, 2);
    if (num > 0){
        res = I2C_Master_ReadReg(UBLOX_M9N, 0xFF, num, gps_data.sentence, I2C_SEL_GPS);
        if (gps_data.sentence[0] != '$'){
                return 10;
        }
#ifdef __DEBUG__
    uart_write_DEBUG(gps_data.sentence,UART_NYX);
    uart_write_DEBUG("\r\n",UART_NYX);
#endif
        return res;
    }
    return 10;
}

uint8_t parseNMEA(void){
    char lat[12] = {0};
    char lng[12] = {0};
    char alt[7] = {0};
    const char NMEA_delimiter[2] = ",";
    char * token = strtoke(gps_data.sentence, NMEA_delimiter);

    uint8_t i = 0;
    for (i = 0; token != NULL; i++) {
        switch (i) {
        case 0:
            break;
        case 1:
            strcpy(gps_data.timestamp, token);
            break;
        case 2:
            strcpy(lat, token);
            break;
        case 3:
            strcpy(gps_data.NS, token);
            break;
        case 4:
            strcpy(lng, token);
            break;
        case 5:
            strcpy(gps_data.EW, token);
            break;
        case 6:
            strcpy(gps_data.quality, token);
            break;
        case 7:
            strcpy(gps_data.satellites, token);
            break;
        case 8:
            strcpy(gps_data.HDOP, token);
            break;
        case 9:
            strcpy(alt, token);
            break;
        case 11:
            strcpy(gps_data.sep, token);
            break;
        }
        token = strtoke(NULL, NMEA_delimiter);
    }
    if (i<11){ //If the number of fields parsed is less than 11. Return error.
        gps_loss_count++;
        if (gps_loss_count > GPS_LOSS_COUNT_THR){
            init_gps_data();
        }
        return 1;
    }
    if ((gps_data.quality[0] == '1') || (gps_data.quality[0] == '2') || (gps_data.quality[0] == '4') || (gps_data.quality[0] == '5')){
        gps_data.latitude = coorsAtol(lat, gps_data.NS);
        gps_data.longtitude = coorsAtol(lng, gps_data.EW);
        gps_data.altitude = altAtol(alt);
        gps_loss_count = 0;
    }
    else if (gps_data.quality[0] == '0'){
        gps_loss_count++;
        if (gps_loss_count > GPS_LOSS_COUNT_THR){
            init_gps_data();
        }
    }
    memset(gps_data.sentence, 0, 75);
    return 0;
}


static void calcChecksum(messageCFG_t *msg){
    msg->checksumA = 0;
    msg->checksumB = 0;

    msg->checksumA += msg->cls;
    msg->checksumB += msg->checksumA;

    msg->checksumA += msg->id;
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len & 0xFF);
    msg->checksumB += msg->checksumA;

    msg->checksumA += (msg->len >> 8);
    msg->checksumB += msg->checksumA;

    uint8_t i;
    for (i=0; i < msg->len; i++)
    {
        msg->checksumA += msg->payload[i];
        msg->checksumB += msg->checksumA;
    }
}

uint8_t sendI2Cmessage(uint8_t msg_len){
    uint8_t message[30] = {0};
    message[0] = UBX_SYNCH_2;
    message[1] = config_message.cls;
    message[2] = config_message.id;
    message[3] = (config_message.len & 0xFF);
    message[4] = (config_message.len >> 8);
    uint8_t i;
    for ( i=0 ; i < config_message.len ; i++){
        message[5+i] = config_message.payload[i];
    }
    message[5+i] = config_message.checksumA;
    message[6+i] = config_message.checksumB;
    return I2C_Master_WriteReg(UBLOX_M9N, UBX_SYNCH_1, message, msg_len, I2C_SEL_GPS);
}

uint8_t setPortOutput(uint8_t portSelect, uint8_t streamSettings){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id =  UBX_CFG_PRT;
    config_message.len = 20;
    uint8_t payloadCfg[20] = {0};
    payloadCfg[4] = 0x84;
    payloadCfg[12] = 0x23;
    payloadCfg[14] = streamSettings;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);
}

uint8_t ubloxSaveConfiguration(void){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id =  UBX_CFG_CFG;
    config_message.len = 12;
    uint8_t payloadCfg[12] = {0};
    payloadCfg[4] = 0xFF;
    payloadCfg[5] = 0xFF;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);
}

uint8_t ubloxLoadConfiguration(void){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id =  UBX_CFG_CFG;
    config_message.len = 12;
    uint8_t payloadCfg[12] = {0};
    payloadCfg[8] = 0xFF;
    payloadCfg[9] = 0xFF;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);
}

uint8_t ubloxResetConfiguration(void){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id =  UBX_CFG_CFG;
    config_message.len = 12;
    uint8_t payloadCfg[12] = {0};
    payloadCfg[0] = 0xFF;
    payloadCfg[1] = 0xFF;
    payloadCfg[8] = 0xFF;
    payloadCfg[9] = 0xFF;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);
}

uint8_t getPortSettings(uint8_t portID, uint8_t *rx_mes){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_PRT;
    config_message.len = 1;
    uint8_t payloadCfg[1] = {0};
    payloadCfg[0] = portID;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    uint8_t message[30] = {0};
    message[0] = UBX_SYNCH_2;
    message[1] = config_message.cls;
    message[2] = config_message.id;
    message[3] = (config_message.len & 0xFF);
    message[4] = (config_message.len >> 8);
    message[5] = payloadCfg[0];
    message[6] = config_message.checksumA;
    message[7] = config_message.checksumB;
    return I2C_Master_WriteMessUblox(UBLOX_M9N, UBX_SYNCH_1, message, rx_mes, 8, 28, I2C_SEL_GPS);
}

uint8_t getMessageSettings(uint8_t msgClass, uint8_t msgID, uint8_t *rx_mes){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_MSG;
    config_message.len = 2;
    uint8_t payloadCfg[2] = {0};
    payloadCfg[0] = msgClass;
    payloadCfg[1] = msgID;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    uint8_t message[30] = {0};
    message[0] = UBX_SYNCH_2;
    message[1] = config_message.cls;
    message[2] = config_message.id;
    message[3] = (config_message.len & 0xFF);
    message[4] = (config_message.len >> 8);
    message[5] = payloadCfg[0];
    message[6] = payloadCfg[1];
    message[7] = config_message.checksumA;
    message[8] = config_message.checksumB;
    return I2C_Master_WriteMessUblox(UBLOX_M9N, UBX_SYNCH_1, message, rx_mes, 9, 11, I2C_SEL_GPS);
}

uint8_t configureNMEA(uint8_t msgClass, uint8_t msgID, uint8_t rate, uint8_t portID){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_MSG;
    config_message.len = 8;
    uint8_t payloadCfg[8] = {0};
    payloadCfg[0] = msgClass;
    payloadCfg[1] = msgID;
    payloadCfg[2] = rate;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);
}

uint8_t powerOffWithInterrupt(uint32_t durationInMs, uint32_t wakeupSources){
    config_message.cls = UBX_CLASS_RXM;
    config_message.id = UBX_RXM_PMREQ;
    config_message.len = 16;
    uint8_t payloadCfg[16] = {0};
    payloadCfg[0] = 0x00; // message version
    // bytes 1-3 are reserved - and must be set to zero
    payloadCfg[1] = 0x00;
    payloadCfg[2] = 0x00;
    payloadCfg[3] = 0x00;
    payloadCfg[4] = (durationInMs >> (8 * 0)) & 0xff;
    payloadCfg[5] = (durationInMs >> (8 * 1)) & 0xff;
    payloadCfg[6] = (durationInMs >> (8 * 2)) & 0xff;
    payloadCfg[7] = (durationInMs >> (8 * 3)) & 0xff;
    payloadCfg[8] = 0x06;
    payloadCfg[9] = 0x00;
    payloadCfg[10] = 0x00;
    payloadCfg[11] = 0x00;
    payloadCfg[12] = (wakeupSources >> (8 * 0)) & 0xff;
    payloadCfg[13] = (wakeupSources >> (8 * 1)) & 0xff;
    payloadCfg[14] = (wakeupSources >> (8 * 2)) & 0xff;
    payloadCfg[15] = (wakeupSources >> (8 * 3)) & 0xff;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);

}

uint8_t setPowerSaveMode(uint8_t mode){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_RXM;
    config_message.len = 2;
    uint8_t payloadCfg[2] = {0};
    payloadCfg[0] = 0;
    payloadCfg[1] = mode;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);
}

uint8_t powerModeGet(void){
    uint8_t res = 0;
    uint8_t data[10] = {0};
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_RXM;
    config_message.len = 0;
    uint8_t message[8] = {0};
    message[0] = UBX_SYNCH_2;
    message[1] = config_message.cls;
    message[2] = config_message.id;
    message[3] = (config_message.len & 0xFF);
    message[4] = (config_message.len >> 8);
    message[5] = config_message.checksumA;
    message[6] = config_message.checksumB;
    res = I2C_Master_WriteMessUblox(UBLOX_M9N, UBX_SYNCH_1, message, data, 7, 9, I2C_SEL_GPS);
    if (res == 8)return res;
    return data[1];
}


uint8_t createBackup(void){
    config_message.cls = UBX_CLASS_UPD;
    config_message.id = 0x14;
    config_message.len = 4;
    uint8_t payloadCfg[4] = {0};
    payloadCfg[0] = 0;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);
}

uint8_t restoreBackupData(void){
    config_message.cls = UBX_CLASS_UPD;
    config_message.id = 0x14;
    config_message.len = 0;
    uint8_t payloadCfg[1] = {0};
    payloadCfg[1] = 0;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);
}

uint8_t resetReceiver(uint16_t startSelect, uint8_t start_stop){
    config_message.cls = UBX_CLASS_CFG;
    config_message.id = UBX_CFG_RST;
    config_message.len = 4;
    uint8_t payloadCfg[4] = {0};
    payloadCfg[0] = startSelect & 0xFF;
    payloadCfg[1] = (startSelect >> 8);
    payloadCfg[3] = start_stop;
    payloadCfg[4] = 0;
    config_message.payload = payloadCfg;
    calcChecksum(&config_message);
    return sendI2Cmessage(config_message.len + 7);
}

void init_gps_data(void){
    memset(gps_data.EW, 0, 1);
    memset(gps_data.HDOP, 0, 5);
    memset(gps_data.NS, 0, 1);
    gps_data.altitude = 0;
    gps_data.latitude = 0;
    gps_data.longtitude = 0;
    memset(gps_data.quality, 0, 1);
    memset(gps_data.satellites, 0, 2);
    memset(gps_data.sentence, 0, 75);
    memset(gps_data.sep, 0, 6);
    memset(gps_data.timestamp, 0, 9);
}



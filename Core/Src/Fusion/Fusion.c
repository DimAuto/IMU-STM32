/*
 * Fusion.c
 *
 *  Created on: Jun 19, 2023
 *      Author: dkalaitzakis
 */

#include "Fusion.h"
#include "../uart.h"
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include "../helpers.h"
#include <math.h>
#include "cmsis_os2.h"
#include "../lsm6_gyro.h"
#include "../../Inc/main.h"

#define SAMPLE_PERIOD (0.034f)
#define SAMPLE_RATE (100)

const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
static FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
//const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
//const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

const FusionMatrix softIronMatrix = {0.1835f, 0.0053f, -0.0027f, 0.0053f, 0.1878, -0.0007f, -0.0027f, -0.0007f, 0.1786f};
const FusionVector hardIronOffset = {0.9485f, 0.0f, 80.0f};

static uint32_t prv_tick = 0;

static clock_t timestamp = 0;
static clock_t previousTimestamp = 0;
static uint32_t update_duration = 0;

FusionAhrs ahrs;
FusionOffset offset;

void setGyroOffset(gyro_data_t values){
	gyroscopeOffset.array[0] = values.gyro_x;
	gyroscopeOffset.array[1] = values.gyro_y;
	gyroscopeOffset.array[2] = values.gyro_z;
}

/* Initialize Fusion algorithm. */
void FusionInit(void){
	gyro_data_t values = {0};
	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);
	const FusionAhrsSettings settings = {
			.convention = FusionConventionNwu,
			.gain = 0.5f,
			.accelerationRejection = 10.0f,
			.magneticRejection = 10.0f,
			.rejectionTimeout = 30 * SAMPLE_RATE, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);
//	if (!Flash_isWritten (GYRO_OFFSET_ADDR)){	// Check if the specific memory addr is written, in order not to cause HRDFAULT
	Flash_Read_CalTable(GYRO_OFFSET_ADDR, &values);
	setGyroOffset(values);
//	}
}

/* Calculate angle based only on Accelerometer and gyroscope.*/
void FusionCalcAngle(mems_data_t *memsData, FusionEuler *output_angles){
	FusionVector gyroscope = {memsData->gyro.gyro_x, memsData->gyro.gyro_y, memsData->gyro.gyro_z};
	const FusionVector accelerometer = {memsData->acc.acc_x, memsData->acc.acc_y, memsData->acc.acc_z};
	gyroscope = FusionVectorSubtract(gyroscope, gyroscopeOffset);
#ifndef GYRO_TS
	float delta = (float)(memsData->timestamp - prv_tick) / 1000.0f;
	prv_tick = memsData->timestamp;
#else
	float delta = (float) ( memsData->timestamp - previousTimestamp) * (float) GYRO_TIMESTAMP_LSB_USEC / (float) 1000000;
	previousTimestamp = memsData->timestamp;
#endif
//	delta += 0.006; //Add a const offset.
	if ((delta >= MEMS_SR_SEC - 7) && (delta <= MEMS_SR_SEC + 7)){
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, delta);
	}
//
//	uint8_t text[20] = {0};
//	sprintf(text, "%f\r\n,", delta);
//	uart_write_debug(text, 20);

	*output_angles = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	if (output_angles->angle.yaw < 0){
		output_angles->angle.yaw += 360;
	}
	if (output_angles->angle.roll < 0){
		output_angles->angle.roll += 360;
	}
	if (output_angles->angle.pitch < 0){
		output_angles->angle.pitch += 360;
	}
	//	const FusionVect = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
}

/* Calculate heading based on all three sensors.*/
void FusionCalcHeading(mems_data_t *memsData, FusionEuler *output_angles){
	FusionVector gyroscope = {memsData->gyro.gyro_x, memsData->gyro.gyro_y, memsData->gyro.gyro_z};
	FusionVector accelerometer = {memsData->acc.acc_x, memsData->acc.acc_y, memsData->acc.acc_z};
	FusionVector magnetometer = {memsData->magn.magn_x, memsData->magn.magn_y, memsData->magn.magn_z}; // replace this with actual magnetometer data in arbitrary units

	// Apply calibration
	gyroscope = FusionVectorSubtract(gyroscope, gyroscopeOffset);
//	accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
	magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

	// Update gyroscope offset correction algorithm
	gyroscope = FusionOffsetUpdate(&offset, gyroscope);

#ifndef GYRO_TS
	float delta = (float)(memsData->timestamp - prv_tick) / 1000.0f;
	prv_tick = memsData->timestamp;
#else
	float delta = (float) ( memsData->timestamp - previousTimestamp) * (float) GYRO_TIMESTAMP_LSB_USEC / (float) 1000000;
	previousTimestamp = memsData->timestamp;
#endif
	// Update gyroscope AHRS algorithm
	if ((delta >= MEMS_SR_SEC - 7) && (delta <= MEMS_SR_SEC + 7)){
		FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta);
	}

	// Print algorithm outputs
	*output_angles = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	if (output_angles->angle.yaw < 0){
		output_angles->angle.yaw += 360;
	}
	if (output_angles->angle.roll < 0){
		output_angles->angle.roll += 360;
	}
	if (output_angles->angle.pitch < 0){
		output_angles->angle.pitch += 360;
	}
//	const FusionVector earth = FusionAhrsGetEarthAcceleration(&ahrs);
}

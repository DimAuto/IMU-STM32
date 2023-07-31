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

#define SAMPLE_PERIOD (0.034f)
#define SAMPLE_RATE (50)

const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
//const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
//const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

const FusionMatrix softIronMatrix = {0.1835f, 0.0053f, -0.0027f, 0.0053f, 0.1878, -0.0007f, -0.0027f, -0.0007f, 0.1786f};
const FusionVector hardIronOffset = {0.9485f, 0.0f, 80.0f};


FusionAhrs ahrs;
FusionOffset offset;

void setGyroOffset(gyro_data_t values){
	gyroscopeOffset.array[0] = values.gyro_x;
	gyroscopeOffset.array[1] = values.gyro_y;
	gyroscopeOffset.array[2] = values.gyro_z;
}

/* Initialize Fusion algorithm. */
void FusionInit(void){
	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);
	const FusionAhrsSettings settings = {
			.convention = FusionConventionNwu,
			.gain = 0.5f,
			.accelerationRejection = 0.0f,
			.magneticRejection = 20.0f,
			.rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);
}

/* Calculate angle based only on Accelerometer and gyroscope.*/
void FusionCalcAngle(mems_data_t *memsData, FusionEuler *output_angles){
	FusionVector gyroscope = {memsData->gyro.gyro_x, memsData->gyro.gyro_y, memsData->gyro.gyro_z};
	const FusionVector accelerometer = {memsData->acc.acc_x, memsData->acc.acc_y, memsData->acc.acc_z};

	gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);

	FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
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
	const clock_t timestamp = memsData->timestamp; // timestamp taken from LSM6DRX gyroscope.
	FusionVector gyroscope = {memsData->gyro.gyro_x, memsData->gyro.gyro_y, memsData->gyro.gyro_z};
	FusionVector accelerometer = {memsData->acc.acc_x, memsData->acc.acc_y, memsData->acc.acc_z};
	FusionVector magnetometer = {memsData->magn.magn_x, memsData->magn.magn_y, memsData->magn.magn_z}; // replace this with actual magnetometer data in arbitrary units

	// Apply calibration
	gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
	accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
	magnetometer = FusionCalibrationMagnetic(magnetometer, softIronMatrix, hardIronOffset);

	// Update gyroscope offset correction algorithm
	gyroscope = FusionOffsetUpdate(&offset, gyroscope);

	// Calculate delta time (in seconds) to account for gyroscope sample clock error
	static clock_t previousTimestamp;
	const float deltaTime = (float) (timestamp - previousTimestamp) / (float) CLOCKS_PER_SEC;
	previousTimestamp = timestamp;


	// Update gyroscope AHRS algorithm
	FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, 0.014);

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

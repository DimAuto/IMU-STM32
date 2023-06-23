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

#define SAMPLE_PERIOD (0.0138f)
#define SAMPLE_RATE (70)

const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

FusionAhrs ahrs;
FusionOffset offset;

/* Initialize Fusion algorithm. */
void FusionInit(void){
	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);
	const FusionAhrsSettings settings = {
			.convention = FusionConventionNwu,
			.gain = 0.5f,
			.accelerationRejection = 10.0f,
			.magneticRejection = 20.0f,
			.rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);

}

/* Calculate angle based only on Accelerometer and gyroscope.*/
void FusionCalcAngle(mems_data_t *memsData, FusionEuler *output_angles){
	const FusionVector gyroscope = {memsData->gyro_x, memsData->gyro_y, memsData->gyro_z};
	const FusionVector accelerometer = {memsData->acc_x, memsData->acc_y, memsData->acc_z};

	FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

	*output_angles = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
	//	const FusionVect = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
}

/* Calculate heading based on all three sensors.*/
void FusionCalcHeading(mems_data_t *memsData, FusionEuler *output_angles){
	const clock_t timestamp = memsData->timestamp; // timestamp taken from LSM6DRX gyroscope.
	FusionVector gyroscope = {memsData->gyro_x, memsData->gyro_y, memsData->gyro_z};
	FusionVector accelerometer = {memsData->acc_x, memsData->acc_y, memsData->acc_z};
	FusionVector magnetometer = {memsData->magn_x, memsData->magn_y, memsData->magn_z}; // replace this with actual magnetometer data in arbitrary units

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
	FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, deltaTime);

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

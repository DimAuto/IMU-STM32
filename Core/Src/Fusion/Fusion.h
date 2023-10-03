/**
 * @file Fusion.h
 * @author Seb Madgwick
 * @brief Main header file for the Fusion library.  This is the only file that
 * needs to be included when using the library.
 */

#ifndef FUSION_H
#define FUSION_H

#include "../lsm6_gyro.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "FusionAhrs.h"
#include "FusionAxes.h"
#include "FusionCalibration.h"
#include "FusionCompass.h"
#include "FusionConvention.h"
#include "FusionMath.h"
#include "FusionOffset.h"


#define GYRO_TIMESTAMP_LSB_USEC 25


void FusionInit(void);

void FusionCalcAngle(mems_data_t *memsData, FusionEuler *output_angles);

void FusionCalcHeading(mems_data_t *memsData, FusionEuler *output_angles);

#ifdef __cplusplus
}
#endif

#endif
//------------------------------------------------------------------------------
// End of file

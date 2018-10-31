/*
 * This file is part of quaternion-based displayIMU C/C++/QT code base
 * (https://github.com/ssymeonidis/displayIMU.git)
 * Copyright (c) 2018 Simeon Symeonidis (formerly Sensor Management Real
 * Time (SMRT) Processing Solutions)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _IMU_CORE_H
#define _IMU_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>
#if IMU_USE_PTHREAD
#include <pthread.h>
#endif
#include "IMU_type.h"

// define status codes
#define IMU_CORE_FNC_DISABLED       1

// define error codes
#define IMU_CORE_INST_OVERFLOW     -1
#define IMU_CORE_BAD_INST          -2
#define IMU_CORE_FAILED_MUTEX      -3

// define constants
#define IMU_CORE_10USEC_TO_SEC      0.00001

// configuration structure definition
typedef struct {
  unsigned char        enable;          // enable IMU core capability
  unsigned char        isGyro;          // enable gyroscope data
  unsigned char        isAccl;          // enable accelerometer data
  unsigned char        isMagn;          // enable magnetometer data
  unsigned char        isStable;        // enable detection of no movement
  unsigned char        isFOM;           // enable weight based on FOM
  unsigned char        isTran;          // enable translational estimate
  unsigned char        isPredict;       // enable extrapolation of estim
  float                gScale;          // scale to covert to rad/sec
  float                aWeight;         // accelerometer IMU weight
  float                aMag;            // gravity magnitude
  float                aMagThresh;      // gravity magnitude error thresh
  float                mWeight;         // magnetometer IMU weight
  float                mMag;            // magnetic north magn  
  float                mMagThresh;      // magnetic north magn error thres
  float                mAng;            // magnetic north angle
  float                mAngThresh;      // magnetic north angle error thresh
  float                tranAlpha;       // translational accleration alpha
} IMU_core_config;

// subsystem state structure definition
typedef struct {
  int                  status;          // captures last datum status
  float                t;               // last datum time
  float                q[4];            // current quaterion
  float                aTran[3];        // last acceleration estimate
  float                mInit[3];        // initial magnetometer value
  unsigned char        aReset;          // accelerometer reset signal
  unsigned char        mReset;          // magnetometer reset signal
} IMU_core_state;

// core datum internal state
typedef enum {
  IMU_core_enum_unitialized  = 1,
  IMU_core_enum_zeroed_accl  = 2,
  IMU_core_enum_zeroed_magn  = 3,
  IMU_core_enum_zeroed_save  = 4,
  IMU_core_enum_zeroed_both  = 5,
  IMU_core_enum_zeroed_gyro  = 6,
  IMU_core_enum_normal_op    = 7,
  IMU_core_enum_no_weight    = 9,
} IMU_core_enum;
  
// data structure access functions
int IMU_core_init     (uint16_t *id, IMU_core_config **config);
int IMU_core_getConfig (uint16_t id, IMU_core_config **config);
int IMU_core_getState  (uint16_t id, IMU_core_state  **state);

// state update functions
int IMU_core_reset     (uint16_t id);
int IMU_core_datum     (uint16_t id, IMU_datum*, IMU_core_FOM*);
int IMU_core_data3     (uint16_t id, IMU_data3*, IMU_core_FOM*);

// state estimation functions
int IMU_core_estmQuat  (uint16_t id, uint32_t t, float* estm);
int IMU_core_estmAccl  (uint16_t id, uint32_t t, float* estm);


#ifdef __cplusplus
}
#endif

#endif

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
#define IMU_CORE_ZEROED_ACCL        2
#define IMU_CORE_ZEROED_MAGN        3
#define IMU_CORE_ZEROED_SAVE        4
#define IMU_CORE_ZEROED_BOTH        5
#define IMU_CORE_ZEROED_GYRO        6
#define IMU_CORE_NO_WEIGHT          7
#define IMU_CORE_GYRO_STABLE        8
#define IMU_CORE_NORMAL_OP          9

// define error codes
#define IMU_CORE_INST_OVERFLOW     -1
#define IMU_CORE_BAD_INST          -2
#define IMU_CORE_FAILED_MUTEX      -3


// configuration structure definition
typedef struct {
  unsigned char        enable;          // enable IMU core capability
  unsigned char        isGyro;          // enable gyroscope data
  unsigned char        isAccl;          // enable accelerometer data
  unsigned char        isMagn;          // enable magnetometer data
  unsigned char        isStable;        // enable detection of no movement
  unsigned char        isFOM;           // enable weight based on FOM
  unsigned char        isPos;           // enable acceleration estimate
  unsigned char        isPredict;       // enable extrapolation of estim
  float                gThresh;         // no motion threshold value
  float                gThreshTime;     // no motion threhsold time
  float                aWeight;         // accelerometer IMU weight
  float                aMag;            // gravity magnitude
  float                aMagThresh;      // gravity magnitude error thresh
  float                mWeight;         // magnetometer IMU weight
  float                mMag;            // magnetic north magn  
  float                mMagThresh;      // magnetic north magn error thres
  float                mAng;            // magnetic north angle
  float                mAngThresh;      // magnetic north angle error thresh
  float                posAlpha;        // position accleration alpha
} IMU_core_config;

// subsystem state structure definition
typedef struct {
  float                t;              // last datum time
  float                tMove;          // last "unstable" time
  float                q[4];           // current quaterion
  float                A[3];           // last acceleration estimate
  float                mInit[3];       // initial magnetometer value
  unsigned char        aReset;         // accelerometer reset signal
  unsigned char        mReset;         // magnetometer reset signal
  unsigned char        estmValid;      // flag to insure valid state
  int                  status;         // captures last datum status
#if IMU_USE_PTHREAD
  pthread_mutex_t      lock;           // mutex (async operation)
#endif
} IMU_core_state;


// data structure access functions
int IMU_core_init     (uint16_t *id, IMU_core_config **config);
int IMU_core_getConfig (uint16_t id, IMU_core_config **config);
int IMU_core_getState  (uint16_t id, IMU_core_state  **state);

// state update functions
int IMU_core_reset     (uint16_t id);
int IMU_core_datum     (uint16_t id, IMU_datum*, IMU_core_FOM*);
int IMU_core_data3     (uint16_t id, IMU_data3*, IMU_core_FOM*);

// state estimation functions
int IMU_core_estmQuat  (uint16_t id, float t, float* estm);
int IMU_core_estmAccl  (uint16_t id, float t, float* estm);


#ifdef __cplusplus
}
#endif

#endif

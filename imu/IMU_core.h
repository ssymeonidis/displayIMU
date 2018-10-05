/*
 * This file is part of quaternion-based displayIMU C/C++/QT code base
 * (https://github.com/ssymeonidis/displayIMU.git)
 * Copyright (c) 2018 Simeon Symeonidis (formerly Sensor Management Real
 * Time (SMRT) Processing Solutions
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
#include "IMU_FOM.h"

// define status codes
#define IMU_CORE_GYRO_STABLE     1

// define error codes
#define IMU_CORE_INST_OVERFLOW  -1
#define IMU_CORE_BAD_INST       -2


// define the configuration structure (values tuned for a part)
typedef struct {
  unsigned char        enable;          // enable IMU core capability
  unsigned char        isGyro;          // enable gyroscope data
  unsigned char        isAccl;          // enable accelerometer data
  unsigned char        isMagn;          // enable magnetometer data
  unsigned char        isStable;        // enable detection of no movement
  unsigned char        isFOM;           // enable weight based on FOM
  unsigned char        isMove;          // enable acceleration estimate
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
  float                moveAlpha;       // acceleration estimate alpha
} IMU_core_config;

// define internal state struct (captures internal IMU state)
typedef struct {
  float                t;              // last datum time
  float                t_move;         // last "unstable" time
  float                SEq[4];         // current quaterion
  float                A[3];           // last acceleration estimate
  float                a[3];           // last acclerometer datum
  unsigned char        aReset;         // accelerometer reset signal
  unsigned char        mReset;         // magnetometer reset signal
} IMU_core_state;


// data structure access functions
int IMU_core_init      (uint16_t *id, IMU_core_config **config);
int IMU_core_getConfig (uint16_t id,  IMU_core_config **config);
int IMU_core_getState  (uint16_t id,  IMU_core_state  **state);

// general update/command operation functions 
int IMU_core_reset   (uint16_t id);
int IMU_core_zero    (uint16_t id, float t, float *a, float *m);
int IMU_core_newGyro (uint16_t id, float t, float *g, IMU_FOM_core*);
int IMU_core_newAccl (uint16_t id, float t, float *a, IMU_FOM_core*);
int IMU_core_newMagn (uint16_t id, float t, float *m, IMU_FOM_core*);
int IMU_core_newAll  (uint16_t id, float t, float *g, float *a, float *m,
                      IMU_FOM_core FOM[3]);

// state estimateion functions
int IMU_core_estmQuat (uint16_t id, float* estm);
int IMU_core_estmAccl (uint16_t id, float* estm);


#ifdef __cplusplus
}
#endif

#endif

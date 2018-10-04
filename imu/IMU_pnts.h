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

#ifndef _IMU_PNTS_H
#define _IMU_PNTS_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>

// define error codes
#define IMU_PNTS_INST_OVERFLOW -1
#define IMU_PNTS_BAD_INST      -2
#define IMU_PNTS_EMPTY_TABLE   -3


// define configuration structure
typedef struct {
  uint8_t                enable;          // to disable entire function
  uint8_t                isAccl;          // process accelerometer data
  uint8_t                isMagn;          // process magnetometer data
  float                  gAlpha;          // mean/std calc filter value 
  float                  gThreshVal;      // no motion threshold value
  float                  gThreshTime;     // no motion threhsold time
  float                  aAlpha;          // accelerometer filter value
  float                  aThresh;         // no motion threshold value
  float                  mAlpha;          // magnetometer filter value
  float                  mThresh;         // no motion threshold value
} IMU_pnts_config;

// point collection internal state
typedef enum {
  IMU_pnts_enum_stop    = 0,
  IMU_pnts_enum_reset   = 1,
  IMU_pnts_enum_moving  = 2,
  IMU_pnts_enum_stable  = 3
} IMU_pnts_enum;

// define internal state 
typedef struct {
  IMU_pnts_enum          state;
  uint16_t               numPnts;
  uint16_t	         curPnts;
  uint16_t               index;
  uint8_t                aClock;
  uint8_t                mClock;
  float                  tStable;
  float                  gMean[3];
} IMU_pnts_state;

// define report fo calibration point
typedef struct {
  float                  tStart;
  float                  tEnd;
  float                  gAccum[3];
  float                  gFltr[3];
  float                  aFltr[3]; 
  float                  mFltr[3];
} IMU_pnts_entry;


// data structure access function
int IMU_pnts_init       (uint16_t *id, IMU_pnts_config**);
int IMU_pnts_getConfig  (uint16_t id,  IMU_pnts_config**);
int IMU_pnts_getState   (uint16_t id,  IMU_pnts_state**);

// points table access function
#if IMU_CALIB_TABLE_SIZE > 1
int IMU_pnts_getCount   (uint16_t id, uint16_t *count);
int IMU_pnts_getEntry   (uint16_t id, uint16_t index, IMU_pnts_entry**);
#endif

// general operation functions 
int IMU_pnts_start      (uint16_t id, uint16_t numPnts);
int IMU_pnts_stop       (uint16_t id);
int IMU_pnts_updateGyro (uint16_t id, float t, float *g, IMU_pnts_entry**);
int IMU_pnts_updateAccl (uint16_t id, float t, float *a, IMU_pnts_entry**);
int IMU_pnts_updateMagn (uint16_t id, float t, float *m, IMU_pnts_entry**);
int IMU_pnts_updateAll  (uint16_t id, float t, float *g, float *a, float *m,
                         IMU_pnts_entry**); 


#ifdef __cplusplus
}
#endif

#endif

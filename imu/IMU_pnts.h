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
#include "IMU_type.h"

// define error codes
#define IMU_PNTS_INST_OVERFLOW -1
#define IMU_PNTS_BAD_INST      -2
#define IMU_PNTS_BAD_INDEX     -3


// configuration structure definition
typedef struct {
  uint8_t                enable;          // to disable entire function
  uint8_t                isAccl;          // process accelerometer data
  uint8_t                isMagn;          // process magnetometer data
  float                  gAlpha;          // mean/std calc filter value 
  float                  gThresh;         // no motion threshold value
  float                  gInitTime;       // no motion threhsold time
  float                  gHoldTime;       // no motion minimum hold time
  float                  aAlpha;          // accelerometer filter value
  float                  aThresh;         // no motion threshold value
  float                  mAlpha;          // magnetometer filter value
  float                  mThresh;         // no motion threshold value
} IMU_pnts_config;

// point collection internal state
typedef enum {
  IMU_pnts_enum_stop     = 0,
  IMU_pnts_enum_reset    = 1,
  IMU_pnts_enum_unstable = 2,
  IMU_pnts_enum_stable   = 3
} IMU_pnts_enum;

// subsystem state structure definition
typedef struct IMU_pnts_entry IMU_pnts_entry;
typedef struct {
  IMU_pnts_enum          state;
  uint8_t                firstFrame;
  uint16_t               numPnts;
  uint16_t	         curPnts;
  uint16_t               index;
  uint8_t                aClock;
  uint8_t                mClock;
  IMU_TYPE               tStable;
  float                  gMean[3];
  IMU_pnts_entry         *current;
} IMU_pnts_state;

// define report fo calibration point
struct IMU_pnts_entry{
  IMU_TYPE               tStart;
  IMU_TYPE               tEnd;
  float                  gAccum[3];
  float                  gFltr[3];
  float                  aFltr[3]; 
  float                  mFltr[3];
  uint16_t               gCount;
  uint16_t               aCount;
  uint16_t               mCount;
};


// data structure access function
int IMU_pnts_init     (uint16_t *id, IMU_pnts_config**);
int IMU_pnts_getConfig (uint16_t id, IMU_pnts_config**);
int IMU_pnts_getState  (uint16_t id, IMU_pnts_state**);

// points table access function (use give calib_table_size greater than one)
int IMU_pnts_getCount  (uint16_t id, uint16_t *count);
int IMU_pnts_getEntry  (uint16_t id, uint16_t index, IMU_pnts_entry**);

// general operation functions 
int IMU_pnts_start     (uint16_t id, uint16_t numPnts);
int IMU_pnts_reset     (uint16_t id);
int IMU_pnts_stop      (uint16_t id);
int IMU_pnts_datum     (uint16_t id, IMU_datum*, IMU_pnts_entry**);
int IMU_pnts_data3     (uint16_t id, IMU_data3*, IMU_pnts_entry**);

// alternate asynchronous interface
int IMU_pnts_newGyro   (uint16_t id, IMU_TYPE t, IMU_TYPE *g, IMU_pnts_entry**);
int IMU_pnts_newAccl   (uint16_t id, IMU_TYPE t, IMU_TYPE *a, IMU_pnts_entry**);
int IMU_pnts_newMagn   (uint16_t id, IMU_TYPE t, IMU_TYPE *m, IMU_pnts_entry**);


#ifdef __cplusplus
}
#endif

#endif

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

#ifndef _IMU_STAT_H
#define _IMU_STAT_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>
#include "IMU_core.h"

// define status codes
#define IMU_STAT_FNC_DISABLED       1

// define error codes
#define IMU_STAT_INST_OVERFLOW      -1
#define IMU_STAT_BAD_INST	    -2

// define constants
#define IMU_STAT_10USEC_TO_SEC      0.00001

// define configuration structure
typedef struct {
  unsigned char         enable;          // enable system characterization
  float                 alpha;           // mean/std calc filter value 
} IMU_stat_config;

// define internal state 
typedef struct {
  float                 gBias[3];
  float                 gBiasStd[3];
  float                 aMag;
  float                 aMagFOM;
  float                 aMagStd;
  float                 mMag;
  float                 mMagFOM;
  float                 mMagStd;
  float                 mDot;
  float                 mDotFOM;
  float                 mDotStd;
  uint8_t               gClock;
  uint8_t               aClock;
  uint8_t               mClock;
  uint32_t              tGyro;
  uint32_t              tAccl;
  uint32_t              tMagn;
} IMU_stat_state;


// data structure access function
int IMU_stat_init     (uint16_t *id, IMU_stat_config **config);
int IMU_stat_getConfig (uint16_t id, IMU_stat_config **config);
int IMU_stat_getState  (uint16_t id, IMU_stat_state  **state);

// general operation functions 
int IMU_stat_reset     (uint16_t id);
int IMU_stat_datum     (uint16_t id, IMU_datum*, IMU_core_FOM*, IMU_pnts_enum);
int IMU_stat_data3     (uint16_t id, IMU_data3*, IMU_core_FOM*, IMU_pnts_enum);


#ifdef __cplusplus
}
#endif

#endif

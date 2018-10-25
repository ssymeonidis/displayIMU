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

// define error codes
#define IMU_STAT_INST_OVERFLOW  -1
#define IMU_STAT_BAD_INST	-2


// define configuration structure
typedef struct {
  unsigned char          enable;          // enable system characterization
  unsigned char          isGyro;          // process gyroscope data
  unsigned char          isAccl;          // process accelerometer data
  unsigned char          isMagn;          // process magnetometer data
  float                  gAlpha;          // mean/std calc filter value 
  float                  aAlpha;          // accelerometer filter value
  float                  mAlpha;          // magnetometer filter value
} IMU_stat_config;

// define internal state 
typedef struct {
  float                  gFltr;
  float                  aMag;
  float                  gMag;
  float                  gAng;
} IMU_stat_state;


// data structure access function
int IMU_stat_init      (uint16_t *id, IMU_stat_config **config);
int IMU_stat_getConfig (uint16_t id,  IMU_stat_config **config);
int IMU_stat_getState  (uint16_t id,  IMU_stat_state  **state);

// general operation functions 
int IMU_stat_reset   (uint16_t id);
int IMU_stat_update  (uint16_t id, IMU_core_FOM*, uint16_t size);


#ifdef __cplusplus
}
#endif

#endif

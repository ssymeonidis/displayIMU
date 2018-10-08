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

#ifndef _IMU_RECT_H
#define _IMU_RECT_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>
#include "IMU_type.h"

// define error codes
#define IMU_RECT_INST_OVERFLOW  -1
#define IMU_RECT_BAD_INST       -2


// define the calibration structure (values tuned for each unit)
typedef struct {
  uint8_t        enable;          // enable IMU sensor correction
  float          gBias[3];        // gyroscope biases
  float          gMult[9];        // gyroscope transform matrix 
  float          aBias[3];        // accelerometer biases
  float          aMult[9];        // accelerometer transform matrix 
  float          mBias[3];        // magnetometer biases
  float          mMult[9];        // magnetometer transform matrix
} IMU_rect_config;

// data structure access functions
int IMU_rect_init      (uint16_t *id, IMU_rect_config **config);
int IMU_rect_getConfig  (uint16_t id, IMU_rect_config **config);

// raw data correction functions
int IMU_rect_datum (uint16_t id, IMU_datum*, IMU_TYPE*);
int IMU_rect_data3 (uint16_t id, IMU_data3*, IMU_TYPE*, IMU_TYPE*, IMU_TYPE*);
int IMU_rect_gyro  (uint16_t id, IMU_TYPE *g_raw, IMU_TYPE *g);
int IMU_rect_accl  (uint16_t id, IMU_TYPE *a_raw, IMU_TYPE *a);
int IMU_rect_magn  (uint16_t id, IMU_TYPE *m_raw, IMU_TYPE *m);


#ifdef __cplusplus
}
#endif

#endif

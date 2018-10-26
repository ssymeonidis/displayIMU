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

#ifndef _IMU_TYPE_H
#define _IMU_TYPE_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>

// structure definitions
typedef struct IMU_core_FOM IMU_core_FOM;

// define global datum enumertations
typedef enum {
  IMU_sync             = 0,
  IMU_gyro             = 1,
  IMU_accl             = 2,
  IMU_magn             = 3
} IMU_sensor;
typedef struct {
  IMU_sensor           type;
  uint32_t             t;
  IMU_TYPE             val[3];
} IMU_datum;
typedef struct {
  uint32_t             t;
  IMU_TYPE             g[3];
  IMU_TYPE             a[3];
  IMU_TYPE             m[3];
} IMU_data3;


// define sensor figure of merit
typedef struct {
  float                magSqrd;
  uint8_t              isStable;
} IMU_core_FOM_gyro;
typedef struct {
  float                mag;
  float                magFOM;
  float                delt;
} IMU_core_FOM_accl;
typedef struct {
  float                mag;
  float                magFOM;
  float                ang;
  float                angFOM;
  float                delt;
} IMU_core_FOM_magn; 


// define datum/data3 figure of merit 
typedef union {
  IMU_datum            *datum;
  IMU_data3            *data3;
} IMU_core_pntr;
struct IMU_core_FOM{
  IMU_core_pntr        pntr;
  uint8_t              isValid;
  union {
    IMU_core_FOM_gyro  gyro;
    IMU_core_FOM_accl  accl;
    IMU_core_FOM_magn  magn;
  }                    FOM;
};


// define calibration figure of merit
typedef struct {
  float                calbFOM;
} IMU_calb_FOM;


#ifdef __cplusplus
}
#endif

#endif

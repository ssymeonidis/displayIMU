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

#ifndef _IMU_FOM_H
#define _IMU_FOM_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>


// define datum enumertation type for FOM structure
typedef enum {
  IMU_FOM_all         = 0,
  IMU_FOM_gyro        = 1,
  IMU_FOM_accl        = 2,
  IMU_FOM_magn        = 3
} IMU_FOM_sensor;

// define sensor figure of merit
typedef struct {
  float                magSqrd;
  uint8_t              isStable;
} IMU_FOM_core_gyro;
typedef struct {
  float                mag;
  float                magFOM;
  float                delt;
} IMU_FOM_core_accl;
typedef struct {
  float                mag;
  float                magFOM;
  float                ang;
  float                angFOM;
  float                delt;
} IMU_FOM_core_magn; 

// define figure of merit 
typedef struct {
  IMU_FOM_sensor       type;
  float                t;
  uint8_t              isValid;
  IMU_TYPE             val[3];
  union {
    IMU_FOM_core_gyro  gyro;
    IMU_FOM_core_accl  accl;
    IMU_FOM_core_magn  magn;
  }                    data;
} IMU_FOM_core;


// define figure of merit
typedef struct {
  float                empty;
} IMU_FOM_calb;



#ifdef __cplusplus
}
#endif

#endif

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

#ifndef _IMU_CALIB_AUTO_H
#define _IMU_CALIB_AUTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "IMU_core.h"

// define error codes
#define IMU_CALIB_AUTO_INST_OVERFLOW  -1
#define IMU_CALIB_AUTO_BAD_INST       -2


// define configuration structure
struct IMU_calib_auto_config {
  unsigned char          enable;          // enable system characterization
  unsigned char          isGyro;          // process gyroscope data
  unsigned char          isAccl;          // process accelerometer data
  unsigned char          isMagn;          // process magnetometer data
  float                  gAlpha;          // mean/std calc filter value 
  float                  aAlpha;          // accelerometer filter value
  float                  mAlpha;          // magnetometer filter value
};

// define internal state 
struct IMU_calib_auto_state {
  float                  gFltr;
  float                  aMag;
  float                  gMag;
  float                  gAng;
};


// data structure access function
int IMU_calib_auto_init(
  unsigned short                  *id, 
  struct IMU_calib_auto_config    **config);
int IMU_calib_auto_getConfig(
  unsigned short                  id,  
  struct IMU_calib_auto_config    **config);
int IMU_calib_auto_getState(
  unsigned short                  id,  
  struct IMU_calib_auto_state     **state);

// general operation functions 
int IMU_calib_auto_updateGyro(
  unsigned short                  id, 
  float                           t, 
  float                           *g);
int IMU_calib_auto_updateAccl(
  unsigned short                  id, 
  float                           t, 
  float                           *a);
int IMU_calib_auto_updateMagn(
  unsigned short                  id, 
  float                           t, 
  float                           *m); 
int IMU_calib_auto_updateAll(
  unsigned short                  id,
  float                           t,
  float                           *g,
  float                           *a,
  float                           *m);
int IMU_calib_auto_updateFOM(
  unsigned short                  id,
  struct IMU_core_FOM             *FOM,
  unsigned short                  size);


#ifdef __cplusplus
}
#endif

#endif

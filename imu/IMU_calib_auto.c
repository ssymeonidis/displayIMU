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

// include statements 
#include "IMU_calib_auto.h"

// internally managed structures
struct IMU_calib_auto_config  config [IMU_MAX_INST]; 
struct IMU_calib_auto_state   state  [IMU_MAX_INST];
static unsigned short         numInst = 0;


/******************************************************************************
* function to create new instance
******************************************************************************/

int IMU_calib_auto_init(
  unsigned short                *id, 
  struct IMU_calib_auto_config  **pntr)
{
  // check for device count overflow
  if (numInst >= IMU_MAX_INST)
    return IMU_CALIB_AUTO_INST_OVERFLOW;

  // return inst handle and config struct
  *id   = numInst; 
  *pntr = &config[*id];
  numInst++;
  
  // exit function
  return 0;
}


/******************************************************************************
* function to return instance config pointer
******************************************************************************/

int IMU_calib_auto_getConfig(
  unsigned short                id,
  struct IMU_calib_auto_config  **pntr)
{
  // check for out-of-bounds condition
  if (id > numInst - 1)
    return IMU_CALIB_AUTO_BAD_INST;

  // return state
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_calib_auto_getState( 
  unsigned short                id,  
  struct IMU_calib_auto_state   **pntr)
{
  // check for out-of-bounds condition
  if (id > numInst - 1)
    return IMU_CALIB_AUTO_BAD_INST; 

  // return state
  *pntr = &state[id];
  return 0;
}


/******************************************************************************
* process gyroscope rates
******************************************************************************/

int IMU_calib_auto_updateGyro(
  unsigned short                id,
  float                         t,
  float                         *g)
{
  return 0;
}


/******************************************************************************
* process accelerometer vector
******************************************************************************/

int IMU_calib_auto_updateAccl(
  unsigned short                id, 
  float                         t, 
  float                         *a)
{
  return 0;
}


/******************************************************************************
* process magnetometer vector
******************************************************************************/

int IMU_calib_auto_updateMagn(
  unsigned short                id, 
  float                         t, 
  float                         *m) 
{
  return 0;
}


/******************************************************************************
* process gyroscope, accelerometer, and magnetometer vectors 
******************************************************************************/

int IMU_calib_auto_updateAll(
  unsigned short                id, 
  float                         t, 
  float                         *g, 
  float                         *a, 
  float                         *m)  
{
  return 0;
}


/******************************************************************************
* process gyroscope, accelerometer, and magnetometer vectors
******************************************************************************/

int IMU_calib_auto_updateFOM(
  unsigned short                id,  
  struct IMU_core_FOM           *FOM,
  unsigned short                size)
{
  return 0;
}

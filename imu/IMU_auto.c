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
#include "IMU_auto.h"

// internally managed structures
IMU_auto_config  config [IMU_MAX_INST]; 
IMU_auto_state   state  [IMU_MAX_INST];
uint16_t         numInstAuto = 0;


/******************************************************************************
* function to create new instance
******************************************************************************/

int IMU_auto_init(
  uint16_t                *id, 
  IMU_auto_config         **pntr)
{
  // check for device count overflow
  if (numInstAuto >= IMU_MAX_INST)
    return IMU_AUTO_INST_OVERFLOW;

  // return inst handle and config struct
  *id   = numInstAuto; 
  *pntr = &config[*id];
  numInstAuto++;
  
  // exit function
  return 0;
}


/******************************************************************************
* function to return instance config pointer
******************************************************************************/

int IMU_auto_getConfig(
  uint16_t                id,
  IMU_auto_config         **pntr)
{
  // check for out-of-bounds condition
  if (id > numInstAuto-1)
    return IMU_AUTO_BAD_INST;

  // return state
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_auto_getState( 
  uint16_t                id,  
  IMU_auto_state          **pntr)
{
  // check for out-of-bounds condition
  if (id > numInstAuto-1)
    return IMU_AUTO_BAD_INST; 

  // return state
  *pntr = &state[id];
  return 0;
}


/******************************************************************************
* process gyroscope, accelerometer, and magnetometer vectors
******************************************************************************/

int IMU_auto_reset(
  uint16_t                id)
{
  return 0;
}


/******************************************************************************
* process gyroscope, accelerometer, and magnetometer vectors
******************************************************************************/

int IMU_auto_newFOM(
  uint16_t                id,  
  IMU_FOM_core            *FOM,
  uint16_t                size)
{
  return 0;
}

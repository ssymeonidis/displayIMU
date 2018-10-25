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

// include statements 
#include "IMU_stat.h"

// internally managed structures
static IMU_stat_config  config [IMU_MAX_INST]; 
static IMU_stat_state   state  [IMU_MAX_INST];
static uint16_t         numInst = 0;


/******************************************************************************
* function to create new instance
******************************************************************************/

int IMU_stat_init(
  uint16_t                *id, 
  IMU_stat_config         **pntr)
{
  // check for device count overflow
  if (numInst >= IMU_MAX_INST)
    return IMU_STAT_INST_OVERFLOW;

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

int IMU_stat_getConfig(
  uint16_t                id,
  IMU_stat_config         **pntr)
{
  // check for out-of-bounds condition
  if (id > numInst-1)
    return IMU_STAT_BAD_INST;

  // return state
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_stat_getState( 
  uint16_t                id,  
  IMU_stat_state          **pntr)
{
  // check for out-of-bounds condition
  if (id > numInst-1)
    return IMU_STAT_BAD_INST; 

  // return state
  *pntr = &state[id];
  return 0;
}


/******************************************************************************
* process gyroscope, accelerometer, and magnetometer vectors
******************************************************************************/

int IMU_stat_reset(
  uint16_t                id)
{
  return 0;
}


/******************************************************************************
* process gyroscope, accelerometer, and magnetometer vectors
******************************************************************************/

int IMU_stat_update(
  uint16_t                id,  
  IMU_core_FOM            *FOM,
  uint16_t                size)
{
  return 0;
}

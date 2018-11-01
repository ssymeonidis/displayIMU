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

  // reset instance
  IMU_stat_reset(*id);

  // pass inst handle and config pointer
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
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_STAT_BAD_INST;

  // pass state and exit (no errors)
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
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_STAT_BAD_INST; 

  // pass state and exit (no errors)
  *pntr = &state[id];
  return 0;
}


/******************************************************************************
* process gyroscope, accelerometer, and magnetometer vectors
******************************************************************************/

int IMU_stat_reset(
  uint16_t                id)
{
  state[id].gBias         = 0.0f;
  state[id].aMag          = 0.0f;
  state[id].mMag          = 0.0f;
  state[id].mDot          = 0.0f;
  state[id].gClock        = 1;
  state[id].aClock        = 1;
  state[id].mClock        = 1;
  state[id].tGyro         = 0;
  state[id].tAccl         = 0;
  state[id].tMagn         = 0;

  // exit function (no failures)
  return 0;
}


/******************************************************************************
* collect statistics on datum
******************************************************************************/

int IMU_stat_datum(
  uint16_t                id, 
  IMU_datum               *datum, 
  IMU_core_FOM            *FOM)
{
  return 0;
}


/******************************************************************************
* collect statistics on data3
******************************************************************************/

int IMU_stat_data3(
  uint16_t                id, 
  IMU_data3               *data3, 
  IMU_core_FOM            *FOM)
{
  return 0;
}

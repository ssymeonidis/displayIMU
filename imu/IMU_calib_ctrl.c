/*
 * This file is part of quaternion-based displayIMU C++/QT code base
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
#include "IMU_core.h"
#include "IMU_calib_pnts.h"
#include "IMU_calib_ctrl.h"

// internally managed structures
struct IMU_correct_config     config  [IMU_MAX_INST];
struct IMU_calib_pnts_entry   table   [IMU_MAX_INST][IMU_CALIB_TABLE_SIZE];
static enum IMU_calib_mode    mode    [IMU_MAX_INST];
static unsigned short         numPnts [IMU_MAX_INST];
static unsigned short         numInst = 0;

// internal constants
static const unsigned short   modePnts[2] = {4, 6};


/******************************************************************************
* function to create new instance
******************************************************************************/

int IMU_calib_ctrl_init(unsigned short *id)
{
  // check for device count overflow
  if (IMU_calib_pnts_inst >= IMU_MAX_INST)
    return IMU_CALIB_PNTS_INST_OVERFLOW;

  // return inst handle and config struct
  *id   = IMU_calib_pnts_inst; 
  *pntr = &config[*id];
  IMU_calib_pnts_inst++;
  
  // exit function
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_calib_ctrl_start(
  unsigned short                  id,
  enum IMU_calib_mode             mode_in);
{
  // check for out-of-bounds condition
  if (id > numInst - 1)
    return IMU_CALIB_CTRL_BAD_INST;

  // copy current entry to the table
  numPnts[id]   = 0;
  mode[id]      = mode_in;
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_calib_ctrl_update(
  unsigned short                id, 
  struct IMU_calib_pnts_entry   *pnt)
{
  // check for out-of-bounds condition
  if (id > numInst - 1)
    return IMU_CALIB_CTRL_BAD_INST; 

  // copy current entry to the table
  memcpy(&table[id][numPnts[id]], pnt, sizeof(struct IMU_calib_pnts_entry));
  numPnts[id]++;

  // determine whether enough points were collected
  if (numPnts[id] < modePnts[mode[id]])
    return 0;
 
  // define internal variables
  struct IMU_correct_config  *current;
  unsigned short             i;

  // copy current config structure to local array
  IMU_correct_getConfig(id, &current);
  memcpy(&config[id], current, sizeof(struct IMU_correct_config));
  
  // process completed points table
  config[id].gBias[0]     = 0;
  config[id].gBias[1]     = 0;
  config[id].gBias[2]     = 0;
  config[id].mBias[0]     = 0;
  config[id].mBias[1]     = 0;
  config[id].mBias[2]     = 0;
  for (i=0; i<4; i++) {
    config[id].gBias[0]  += table[id][i].gFltr[0];
    config[id].gBias[1]  += table[id][i].gFltr[1];
    config[id].gBias[2]  += table[id][i].gFltr[2];
    config[id].mBias[0]  += table[id][i].mFltr[0];
    config[id].mBias[1]  += table[id][i].mFltr[1];
    config[id].mBias[2]  += table[id][i].nFltr[2];
  }
    
  return IMU_CALIB_CTRL_UPDATED;
}

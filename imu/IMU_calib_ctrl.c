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
struct IMU_correct_state      config  [IMU_MAX_INST];
struct IMU_calib_pnts_entry   table   [IMU_MAX_INST][IMU_CALIB_TABLE_SIZE];
static unsigned short         numPnts [IMU_MAX_INST];
static unsigned short         numInst = 0;


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

int IMU_calib_ctrl_4pnt_update(unsigned short id, IMU_calib_pnts_entry *pnt)
{
  // check for out-of-bounds condition
  if (id > numInst - 1)
    return IMU_CALIB_CTRL_BAD_INST; 

  // define internal variables
  float                 gAvg[3] = {0, 0, 0};
  float                 mAvg[3] = {0, 0, 0};
  unsigned short        count;
  IMU_calib_pnts_entry  *point;
 
  // verifiy number of collected points
  IMU_calib_ponts_getCount(id, &count);
  if (count != 4)
    return IMU_CALIB_CTRL_WRONG_PNTS_COUNT;
  
  return 0;
}

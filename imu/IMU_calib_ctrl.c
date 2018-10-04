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
#include <string.h>           // memcpy
#include "IMU_calib_ctrl.h"

// internally managed variables
struct IMU_calib_ctrl_state   state[IMU_MAX_INST];     
static unsigned short         numInst = 0;
static const unsigned short   modePnts[2] = {4, 6};


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

void calib_4pnt(unsigned short id)
{
  // define internal variables
  struct IMU_correct_config *correct = &state[id].correct;
  struct IMU_calib_pnts_entry *table = state[id].table;
  int i;

  // process completed points table
  correct->gBias[0]     = 0;
  correct->gBias[1]     = 0;
  correct->gBias[2]     = 0;
  correct->mBias[0]     = 0;
  correct->mBias[1]     = 0;
  correct->mBias[2]     = 0;
  for (i=0; i<4; i++) {
    correct->gBias[0]  += table[i].gFltr[0];
    correct->gBias[1]  += table[i].gFltr[1];
    correct->gBias[2]  += table[i].gFltr[2];
    correct->mBias[0]  += table[i].mFltr[0];
    correct->mBias[1]  += table[i].mFltr[1];
    correct->mBias[2]  += table[i].mFltr[2];
  }
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

void calib_6pnt(unsigned short id)
{ 
  // define internal variables
  struct IMU_correct_config *correct = &state[id].correct;
  struct IMU_calib_pnts_entry *table = state[id].table;
  int i;

  // process completed points table
  correct->gBias[0]     = 0;
  correct->gBias[1]     = 0;
  correct->gBias[2]     = 0;
  correct->aBias[0]     = 0;
  correct->aBias[1]     = 0;
  correct->aBias[2]     = 0;
  correct->mBias[0]     = 0;
  correct->mBias[1]     = 0;
  correct->mBias[2]     = 0;
  for (i=0; i<6; i++) {
    correct->gBias[0]  += table[i].gFltr[0];
    correct->gBias[1]  += table[i].gFltr[1];
    correct->gBias[2]  += table[i].gFltr[2];
    correct->aBias[0]  += table[i].aFltr[0];
    correct->aBias[1]  += table[i].aFltr[1];
    correct->aBias[2]  += table[i].aFltr[2];
    correct->mBias[0]  += table[i].mFltr[0];
    correct->mBias[1]  += table[i].mFltr[1];
    correct->mBias[2]  += table[i].mFltr[2];
  }
}


/******************************************************************************
* function to create new instance
******************************************************************************/

int IMU_calib_ctrl_init(unsigned short *id)
{
  // check for device count overflow
  if (numInst >= IMU_MAX_INST)
    return IMU_CALIB_PNTS_INST_OVERFLOW;

  // return inst handle and config struct
  *id   = numInst; 
  numInst++;
  
  // exit function
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_calib_ctrl_start(
  unsigned short                id,
  enum IMU_calib_ctrl_mode      mode,
  struct IMU_correct_config     *correct,
  struct IMU_core_config        *core)
{
  // check for out-of-bounds condition
  if (id > numInst - 1)
    return IMU_CALIB_CTRL_BAD_INST;

  // copy current entry to the table
  state[id].numPnts             = 0;
  state[id].mode                = mode;
  memcpy(&state[id].correct, correct, sizeof(struct IMU_correct_config));
  memcpy(&state[id].core,    core,    sizeof(struct IMU_core_config));
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_calib_ctrl_update(
  unsigned short                id, 
  struct IMU_calib_pnts_entry   *pntr,
  struct IMU_calib_ctrl_FOM     *FOM)
{
  // check for out-of-bounds condition
  if (id > numInst - 1)
    return IMU_CALIB_CTRL_BAD_INST; 

  // copy current entry to the table
  struct IMU_calib_pnts_entry *entry = &state[id].table[state[id].numPnts]; 
  memcpy(entry, pntr, sizeof(struct IMU_calib_pnts_entry));
  state[id].numPnts++;

  // determine whether enough points were collected
  if (state[id].numPnts < modePnts[state[id].mode])
    return 0;
 
  // perform the specified calibration routine
  if      (state[id].mode == IMU_calib_ctrl_4pnt)
    calib_4pnt(id);
  else if (state[id].mode == IMU_calib_ctrl_6pnt)
    calib_6pnt(id);
  else
    return IMU_CALIB_CTRL_BAD_MODE; 
    
  // exit fuction
  return IMU_CALIB_CTRL_UPDATED;
}


/******************************************************************************
* copy calibration result to IMU_correct structure 
******************************************************************************/

int IMU_calib_ctrl_save(
  unsigned short                id,
  struct IMU_correct_config     *correct)
{
  // check for out-of-bounds condition
  if (id > numInst - 1)
    return IMU_CALIB_CTRL_BAD_INST;

  // copy current entry to the table
  memcpy(correct, &state[id].correct, sizeof(struct IMU_correct_config));
  return 0;
}


/******************************************************************************
* update IMU_correct structure w/ autocal results
******************************************************************************/

int IMU_calib_ctrl_auto(
  unsigned short                id,
  struct IMU_calib_auto_state   *state,
  struct IMU_correct_config     *correct,
  struct IMU_core_config        *core)
{
  return 0;
}

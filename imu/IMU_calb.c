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
#include "IMU_calb.h"

// internally managed variables
IMU_calb_config  config [IMU_MAX_INST];
IMU_calb_state   state  [IMU_MAX_INST];
IMU_pnts_entry   table  [IMU_MAX_INST][IMU_CALB_SIZE]; 
uint16_t         numInstCalb = 0;
uint16_t         modePnts[2] = {4, 6};

// internally defined functions
void calb_4pnt(uint16_t id);
void calb_6pnt(uint16_t id);


/******************************************************************************
* function to create new instance
******************************************************************************/

int IMU_calb_init(
  uint16_t                *id,
  IMU_calb_config         **pntr)
{
  // check for device count overflow
  if (numInstCalb >= IMU_MAX_INST)
    return IMU_CALB_INST_OVERFLOW;

  // return inst handle and config struct
  *id   = numInstCalb; 
  *pntr    = &config[*id];
  numInstCalb++;
  
  // exit function
  return 0;
}


/******************************************************************************
* function to copy config structure
******************************************************************************/

int IMU_calb_getConfig( 
  uint16_t                id,  
  IMU_calb_config         **pntr)
{
  // check for out-of-bounds condition
  if (id >= numInstCalb)
    return IMU_CALB_BAD_INST; 

  // return state
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* function to create new instance
******************************************************************************/

int IMU_calb_setStruct(
  uint16_t                id,
  IMU_rect_config         *rect, 
  IMU_core_config         *core)
{
  // check for device count overflow
  if (id >= numInstCalb)
    return IMU_CALB_BAD_INST;

  // copy rectify and core structures
  memcpy(&state[id].rect_org, rect, sizeof(IMU_rect_config));
  memcpy(&state[id].core_org, core, sizeof(IMU_rect_config)); 
  
  // exit function
  return 0;
}


/******************************************************************************
* function to reset state of calib subsystem
******************************************************************************/

int IMU_calb_reset(
  uint16_t                id)
{
  // check for out-of-bounds condition
  if (id >= numInstCalb)
    return IMU_CALB_BAD_INST;

  // copy current entry to the table
  state[id].numPnts       = 0;
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_calb_start(
  uint16_t                id,
  IMU_calb_mode           mode)
{
  // check for out-of-bounds condition
  if (id >= numInstCalb)
    return IMU_CALB_BAD_INST;

  // copy current entry to the table
  state[id].numPnts       = 0;
  state[id].mode          = mode;
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_calb_pnts(
  uint16_t                id, 
  IMU_pnts_entry          *pntr)
{
  // check for out-of-bounds condition
  if (id >= numInstCalb)
    return IMU_CALB_BAD_INST; 

  // copy current entry to the table
  IMU_pnts_entry *entry = &table[id][state[id].numPnts]; 
  memcpy(entry, pntr, sizeof(IMU_pnts_entry));
  state[id].numPnts++;

  // determine whether enough points were collected
  if (state[id].numPnts < modePnts[state[id].mode])
    return 0;
 
  // perform the specified calibration routine
  if      (state[id].mode == IMU_calb_4pnt)
    calb_4pnt(id);
  else if (state[id].mode == IMU_calb_6pnt)
    calb_6pnt(id);
  else
    return IMU_CALB_BAD_MODE; 
    
  // exit fuction
  return IMU_CALB_UPDATED;
}


/******************************************************************************
* copy calibration result to IMU_correct structure 
******************************************************************************/

int IMU_calb_save(
  uint16_t                id,
  IMU_rect_config         *rect,
  IMU_core_config         *core)
{
  // check for out-of-bounds condition
  if (id >= numInstCalb)
    return IMU_CALB_BAD_INST;

  // copy current entry to the IMU rectify config 
  memcpy(rect, &state[id].rect, sizeof(IMU_rect_config));
  memcpy(core, &state[id].core, sizeof(IMU_core_config));
  return 0;
}


/******************************************************************************
* copy calibration result to IMU_correct structure 
******************************************************************************/

int IMU_calb_status(
  uint16_t                id,
  IMU_calb_FOM            **FOM)
{
  // check for out-of-bounds condition
  if (id >= numInstCalb)
    return IMU_CALB_BAD_INST;

  // determine whether enough points were collected
  if (state[id].numPnts < modePnts[state[id].mode])
    return 0;
  
  // return FOM structure
  *FOM = &state[id].FOM;
  return IMU_CALB_UPDATED;
}


/******************************************************************************
* update IMU_correct structure w/ autocal results
******************************************************************************/

int IMU_calb_auto(
  uint16_t                id,
  IMU_auto_state          *state)
{
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

void calb_4pnt(
  uint16_t                id)
{
  // define internal variables
  IMU_rect_config *rect = &state[id].rect;
  int i;

  // process completed points table
  rect->gBias[0]     = 0;
  rect->gBias[1]     = 0;
  rect->gBias[2]     = 0;
  rect->mBias[0]     = 0;
  rect->mBias[1]     = 0;
  rect->mBias[2]     = 0;
  for (i=0; i<4; i++) {
    rect->gBias[0]  += table[id][i].gFltr[0];
    rect->gBias[1]  += table[id][i].gFltr[1];
    rect->gBias[2]  += table[id][i].gFltr[2];
    rect->mBias[0]  += table[id][i].mFltr[0];
    rect->mBias[1]  += table[id][i].mFltr[1];
    rect->mBias[2]  += table[id][i].mFltr[2];
  }
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

void calb_6pnt(
  uint16_t                id)
{ 
  // define internal variables
  IMU_rect_config *rect = &state[id].rect;
  int i;

  // process completed points table
  rect->gBias[0]     = 0;
  rect->gBias[1]     = 0;
  rect->gBias[2]     = 0;
  rect->aBias[0]     = 0;
  rect->aBias[1]     = 0;
  rect->aBias[2]     = 0;
  rect->mBias[0]     = 0;
  rect->mBias[1]     = 0;
  rect->mBias[2]     = 0;
  for (i=0; i<6; i++) {
    rect->gBias[0]  += table[id][i].gFltr[0];
    rect->gBias[1]  += table[id][i].gFltr[1];
    rect->gBias[2]  += table[id][i].gFltr[2];
    rect->aBias[0]  += table[id][i].aFltr[0];
    rect->aBias[1]  += table[id][i].aFltr[1];
    rect->aBias[2]  += table[id][i].aFltr[2];
    rect->mBias[0]  += table[id][i].mFltr[0];
    rect->mBias[1]  += table[id][i].mFltr[1];
    rect->mBias[2]  += table[id][i].mFltr[2];
  }
}

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

// definitions for increased readability
#define NULL 0

// include statements 
#include "IMU_calib_pnts.h"

// internally managed structures
struct IMU_calib_pnts_config  config [IMU_MAX_INST]; 
struct IMU_calib_pnts_state   state  [IMU_MAX_INST];
struct IMU_calib_pnts_entry   table  [IMU_MAX_INST][IMU_CALIB_TABLE_SIZE];
static unsigned short         IMU_calib_pnts_inst = 0;


/******************************************************************************
* utility function - break stable
******************************************************************************/

inline void break_stable_state(unsigned short id)
{
  state[id].curPnts++;
  state[id].index++;
  if (state[id].index >= IMU_CALIB_TABLE_SIZE)
    state[id].index         = 0;
  if (state[id].curPnts != 0 && state[id].curPnts >= state[id].numPnts)
    state[id].state         = stop;
  else
    state[id].state         = reset; 
}


/******************************************************************************
* function to create new instance
******************************************************************************/

int IMU_calib_pnts_init(
  unsigned short                *id, 
  struct IMU_calib_pnts_config  **pntr)
{
  // check for device count overflow
  if (IMU_calib_pnts_inst >= IMU_MAX_INST)
    return IMU_CALIB_PNTS_INST_OVERFLOW;

  // return inst handle and config struct
  *id   = IMU_calib_pnts_inst; 
  *pntr = &config[*id];
  IMU_calib_pnts_inst++;
  
  // initialize instance state  
  state[*id].numPnts             = 0;
  state[*id].curPnts             = 0;
  state[*id].state               = stop;
  state[*id].index               = 0; 
  state[*id].aClock              = 0;
  state[*id].mClock              = 0;
  
  // exit function
  return 0;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

int IMU_calib_pnts_getState( 
  unsigned short               id,  
  struct IMU_calib_pnts_state  **pntr)
{
  // check for out-of-bounds condition
  if (id > IMU_calib_pnts_inst - 1)
    return IMU_CALIB_PNTS_BAD_INST; 

  // return state
  *pntr = &state[id];
  return 0;
}


/******************************************************************************
* function to return instance last entry pointer
******************************************************************************/

int IMU_calib_pnts_getEntry(
  unsigned short                id, 
  unsigned short                index,
  struct IMU_calib_pnts_entry   **pntr)
{
  // check for out-of-bounds condition
  if (id > IMU_calib_pnts_inst - 1)
    return IMU_CALIB_PNTS_BAD_INST;
  if (state[id].numPnts < 1)
    return IMU_CALIB_PNTS_EMPTY_TABLE;

  // return state
  short i = index - index - 1;
  if (i < 0)
    i += IMU_CALIB_TABLE_SIZE;
  *pntr = &table[id][i];
  return 0;
}


/******************************************************************************
* function to reset instance state
******************************************************************************/

int IMU_calib_pnts_start(
  unsigned short                id,
  unsigned short                numPnts)
{
  // check for out-of-bounds condition
  if (id > IMU_calib_pnts_inst - 1)
    return IMU_CALIB_PNTS_BAD_INST;

  // ininitialize inst state 
  state[id].numPnts             = numPnts;
  state[id].curPnts             = 0;
  state[id].state               = reset;
  state[id].index               = 0; 
  state[id].aClock              = 0;
  state[id].mClock              = 0;
  return 0;
}


/******************************************************************************
* function to reset instance state
******************************************************************************/

int IMU_calib_pnts_stop(
  unsigned short                id)
{
  // check for out-of-bounds condition
  if (id > IMU_calib_pnts_inst - 1)
    return IMU_CALIB_PNTS_BAD_INST;

  // ininitialize inst state
  state[id].state               = stop;
  state[id].aClock              = 0;
  state[id].mClock              = 0;
  return 0;
}


/******************************************************************************
* process gyroscope rates
******************************************************************************/

int IMU_calib_pnts_updateGyro(
  unsigned short                  id,
  float                           t,
  float                           *g,
  struct IMU_calib_pnts_entry     **pntr)
{
  // initialize entry pointer to NULL
  *pntr                 = NULL;

  // check for stop condition
  if (state[id].state == stop)
    return state[id].curPnts;

  // define the variable
  struct IMU_calib_pnts_entry *entry = &table[id][state[id].index];

  // calculate gyroscope deviation (estimates movement w/o bias) 
  float diff;
  float std;
  float alpha           = config[id].gAlpha;
  state[id].gMean[0]    = alpha * g[0] + (1-alpha) * state[id].gMean[0];
  state[id].gMean[1]    = alpha * g[1] + (1-alpha) * state[id].gMean[1];
  state[id].gMean[2]    = alpha * g[2] + (1-alpha) * state[id].gMean[2];
  diff                  = state[id].gMean[0] - g[0];  
  std                   = diff * diff;
  diff                  = state[id].gMean[1] - g[1];  
  std                  += diff * diff;
  diff                  = state[id].gMean[2] - g[2];  
  std                  += diff * diff;

  // determine whether signal meets stability requirements 
  unsigned int stable   = 0;
  if (std > config[id].gThreshVal * config[id].gThreshVal)
    state[id].tStable   = t;
  else if (t - state[id].tStable > config[id].gThreshTime) 
    stable              = 1;

  // state entered upon initialization or stability break 
  if (state[id].state == reset) {
    entry->gAccum[0]    = g[0];
    entry->gAccum[1]    = g[1];
    entry->gAccum[2]    = g[2];
    state[id].tStable   = t;
    entry->tStart       = t;
    state[id].state     = moving;
  } 

  // state occurs prior to reaching "stable" 
  else if (state[id].state == moving) {
    if (!stable) {
      entry->gAccum[0] += g[0];
      entry->gAccum[1] += g[1];
      entry->gAccum[2] += g[2];
      entry->tEnd       = t;
    } else {
      entry->gFltr[0]   = g[0];
      entry->gFltr[1]   = g[1];
      entry->gFltr[2]   = g[2];
      state[id].state   = stable;
      state[id].aClock  = 1;
      state[id].mClock  = 1;
    } 
  }

  // state occurs after reaching "stable" 
  else if (state[id].state == stable) {
    if (stable) {
      entry->gFltr[0]   = alpha * g[0] + (1 - alpha) * entry->gFltr[0];
      entry->gFltr[1]   = alpha * g[1] + (1 - alpha) * entry->gFltr[1]; 
      entry->gFltr[2]   = alpha * g[2] + (1 - alpha) * entry->gFltr[2];
    } else {
      *pntr             = entry; 
      break_stable_state(id);
    }
  } 

  // exit function
  return state[id].curPnts;
}


/******************************************************************************
* process accelerometer vector
******************************************************************************/

int IMU_calib_pnts_updateAccl(
  unsigned short                  id, 
  float                           t, 
  float                           *a,
  struct IMU_calib_pnts_entry     **pntr)
{
  // initialize entry pointer to NULL
  *pntr                 = NULL;

  // verify system state (stable and sensor is enabled)  
  if (!config[id].isAccl || state[id].state != stable)
    return state[id].curPnts;

  // define the variable
  struct IMU_calib_pnts_entry *entry  = &table[id][state[id].index];

  // save accelerometer data
  if (state[id].aClock) {
    entry->aFltr[0]     = a[0];
    entry->aFltr[1]     = a[1];
    entry->aFltr[2]     = a[2];
    state[id].aClock    = 0;
    return state[id].curPnts;
  }

  // check for break-lock or bad data
  float diff            = entry->aFltr[0] - a[0];
  float std             = diff * diff;
  diff                  = entry->aFltr[1] - a[1];
  std                  += diff * diff;
  diff                  = entry->aFltr[2] - a[2];
  std                  += diff * diff;
  if (std > config[id].aThresh * config[id].aThresh) {
    *pntr               = entry; 
    break_stable_state(id);
    return state[id].curPnts;
  }

  // initialize figure of merit
  float alpha           = config[id].aAlpha;
  entry->aFltr[0]       = alpha * a[0] + (1 - alpha) * entry->aFltr[0];
  entry->aFltr[1]       = alpha * a[1] + (1 - alpha) * entry->aFltr[1]; 
  entry->aFltr[2]       = alpha * a[2] + (1 - alpha) * entry->aFltr[2];

  // exit function
  return state[id].curPnts;
}


/******************************************************************************
* process magnetometer vector
******************************************************************************/

int IMU_calib_pnts_updateMagn(
  unsigned short                  id, 
  float                           t, 
  float                           *m, 
  struct IMU_calib_pnts_entry     **pntr)
{
  // initialize entry pointer to NULL
  *pntr                 = NULL;

  // verify system state (stable and sensor is enabled)
  if (!config[id].isMagn || state[id].state != stable)
    return state[id].curPnts;

  // define the variable
  struct IMU_calib_pnts_entry *entry = &table[id][state[id].index];

  // save accelerometer data
  if (state[id].mClock) {
    entry->mFltr[0]     = m[0];
    entry->mFltr[1]     = m[1];
    entry->mFltr[2]     = m[2];
    state[id].mClock    = 0;
    return state[id].curPnts;
  }

  // check for break-lock or bad data
  float diff            = entry->mFltr[0] - m[0];
  float std             = diff * diff;
  diff                  = entry->mFltr[1] - m[1];
  std                  += diff * diff;
  diff                  = entry->mFltr[2] - m[2];
  std                  += diff * diff;
  if (std > config[id].mThresh * config[id].mThresh) {
    *pntr               = entry; 
    break_stable_state(id);
    return state[id].curPnts;
  }

  // initialize figure of merit
  float alpha           = config[id].mAlpha;
  entry->mFltr[0]       = alpha * m[0] + (1 - alpha) * entry->mFltr[0];
  entry->mFltr[1]       = alpha * m[1] + (1 - alpha) * entry->mFltr[1];
  entry->mFltr[2]       = alpha * m[2] + (1 - alpha) * entry->mFltr[2];

  // exit function
  return state[id].curPnts;
}


/******************************************************************************
* process gyroscope, accelerometer, and magnetometer vectors 
******************************************************************************/

int IMU_calib_pnts_updateAll(
  unsigned short                  id, 
  float                           t, 
  float                           *g, 
  float                           *a, 
  float                           *m,  
  struct IMU_calib_pnts_entry     **pntr)
{
  // define local variables
  struct IMU_calib_pnts_entry     *entry;

  // update all three vectors
  IMU_calib_pnts_updateGyro(id, t, g, &entry);
  *pntr = entry; 
  IMU_calib_pnts_updateAccl(id, t, a, &entry); 
  *pntr = (entry != NULL) ? entry : *pntr; 
  IMU_calib_pnts_updateMagn(id, t, m, &entry); 
  *pntr = (entry != NULL) ? entry : *pntr; 
  return state[id].curPnts;
}

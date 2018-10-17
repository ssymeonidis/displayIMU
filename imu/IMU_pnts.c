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

// definitions (increase readability)
#define NULL 0

// include statements 
#include "IMU_pnts.h"

// internally managed structures
static IMU_pnts_config  config [IMU_MAX_INST]; 
static IMU_pnts_state   state  [IMU_MAX_INST];
static IMU_pnts_entry   table  [IMU_MAX_INST][IMU_PNTS_SIZE];
static uint16_t         numInst = 0;

// internally defined functions
static inline IMU_pnts_entry* break_stable(uint16_t id, IMU_TYPE, IMU_TYPE);
static inline float calc_std(float *val1, IMU_TYPE *val2);
static inline void  apply_alpha(float *prev, IMU_TYPE *cur, float alpha);
static inline void  copy_val(float *val1, IMU_TYPE *val2);


/******************************************************************************
* function for creating new instance
******************************************************************************/

int IMU_pnts_init(
  uint16_t                *id, 
  IMU_pnts_config         **pntr)
{
  // check device count overflow
  if (numInst >= IMU_MAX_INST)
    return IMU_PNTS_INST_OVERFLOW;

  // pass inst handle and config pointer
  *id   = numInst; 
  *pntr = &config[*id];
  numInst++;
  
  // initialize instance state
  state[*id].state        = IMU_pnts_enum_stop;
  state[*id].firstFrame   = 1;
  state[*id].numPnts      = 0;
  state[*id].curPnts      = 0;
  state[*id].index        = 0; 
  state[*id].aClock       = 0;
  state[*id].mClock       = 0;
  state[*id].current      = NULL;
  
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* function to return config structure
******************************************************************************/

int IMU_pnts_getConfig( 
  uint16_t                id,  
  IMU_pnts_config         **pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_PNTS_BAD_INST; 

  // pass state and exit (no errors)
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* function to return state structure
******************************************************************************/

int IMU_pnts_getState( 
  uint16_t                id,  
  IMU_pnts_state          **pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_PNTS_BAD_INST; 

  // pass state and exit (no errors)
  *pntr = &state[id];
  return 0;
}


/******************************************************************************
* function to return number of calibration points
******************************************************************************/

int IMU_pnts_getCount(
  uint16_t                id, 
  uint16_t                *count)
{
  // initialize count
  *count = 0;

  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_PNTS_BAD_INST;

  // exit function (no errors)
  *count = state[id].curPnts;
  return 0;
}


/******************************************************************************
* function to return instance last entry pointer
******************************************************************************/

int IMU_pnts_getEntry(
  uint16_t                id, 
  uint16_t                index,
  IMU_pnts_entry          **pntr)
{
  // initialize pointer
  *pntr = NULL;

  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_PNTS_BAD_INST;
  if (index >= state[id].curPnts || index >= IMU_PNTS_SIZE)
    return IMU_PNTS_BAD_INDEX;

  // pass pointer to table entry
  int i = state[id].index - index;
  if (i < 0)
    i += IMU_PNTS_SIZE;
  *pntr = &table[id][i];
 
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* function to start instance state
******************************************************************************/

int IMU_pnts_start(
  uint16_t                id,
  uint16_t                numPnts)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_PNTS_BAD_INST;

  // ininitialize inst state
  state[id].state         = IMU_pnts_enum_reset;
  state[id].numPnts       = numPnts;
  state[id].curPnts       = 0;
  state[id].index         = 0; 
  state[id].aClock        = 0;
  state[id].mClock        = 0;
  state[id].current       = &table[id][0];

  // exit function (no errors)
  return 0;
}


/******************************************************************************
* function to reset instance state
******************************************************************************/

int IMU_pnts_reset(
  uint16_t                id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_PNTS_BAD_INST;

  // ininitialize instance state 
  state[id].state         = IMU_pnts_enum_reset;
  state[id].curPnts       = 0;
  state[id].index         = 0;
  state[id].aClock        = 0;
  state[id].mClock        = 0;
  state[id].current       = &table[id][0];

  // exit function (no errors)
  return 0;
}


/******************************************************************************
* function to reset instance state
******************************************************************************/

int IMU_pnts_stop(
  uint16_t                id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_PNTS_BAD_INST;

  // ininitialize inst state
  state[id].state         = IMU_pnts_enum_stop;
  state[id].numPnts       = 0;
  state[id].curPnts       = 0;
  state[id].index         = 0; 
  state[id].aClock        = 0;
  state[id].mClock        = 0;
  state[id].current       = NULL;

  // exit function (no errors)
  return 0;
}


/******************************************************************************
* initialize state and autocal to known state
******************************************************************************/

int IMU_pnts_datum(
  uint16_t                id,
  IMU_datum               *datum,
  IMU_pnts_entry          **pntr)
{
  // check sensor type and execute
  if      (datum->type == IMU_gyro)
    return IMU_pnts_newGyro(id, datum->t, datum->val, pntr);
  else if (datum->type == IMU_accl)
    return IMU_pnts_newAccl(id, datum->t, datum->val, pntr);
  else if (datum->type == IMU_magn)
    return IMU_pnts_newMagn(id, datum->t, datum->val, pntr);

  // exit function (no errors)
  return 0;
}


/******************************************************************************
* process gyroscope, accelerometer, and magnetometer vectors 
******************************************************************************/

int IMU_pnts_data3(
  uint16_t                id,
  IMU_data3               *data3,
  IMU_pnts_entry          **pntr)
{
  // define local variables
  IMU_pnts_entry          *entry;

  // update all three vectors
  IMU_pnts_newGyro(id, data3->t, data3->g, &entry);
  *pntr = entry; 
  IMU_pnts_newAccl(id, data3->t, data3->a, &entry); 
  *pntr = (entry != NULL) ? entry : *pntr; 
  IMU_pnts_newMagn(id, data3->t, data3->m, &entry); 
  *pntr = (entry != NULL) ? entry : *pntr;
  return state[id].curPnts;
}


/******************************************************************************
* process gyroscope rates
******************************************************************************/

int IMU_pnts_newGyro(
  uint16_t                id,
  IMU_TYPE                t,
  IMU_TYPE                *g,
  IMU_pnts_entry          **pntr)
{
  // initialize output to NULL
  *pntr                   = NULL;

  // check stop condition
  if (state[id].state == IMU_pnts_enum_stop)
    return state[id].curPnts;

  // define the variable
  IMU_pnts_entry *entry   = &table[id][state[id].index];

  // only update gMean given first frame
  if (state[id].firstFrame) {
    state[id].state       = IMU_pnts_enum_unstable;
    state[id].firstFrame  = 0;
    state[id].tStable     = t;
    copy_val(state[id].gMean, g);
    entry->tStart         = t;
    entry->gAccum[0]      = (float)g[0];
    entry->gAccum[1]      = (float)g[1];
    entry->gAccum[2]      = (float)g[2];
    return 0;
  }
  
  // calculate gyroscope deviation (estimates movement w/o bias)
  float std = calc_std(state[id].gMean, g);
  apply_alpha(state[id].gMean, g, config[id].gAlpha);

  // determine whether signal meets stability requirements 
  int       stable        = 0;
  IMU_TYPE  tStable       = state[id].tStable;
  if (std > config[id].gThresh * config[id].gThresh)
    state[id].tStable     = t;
  else if (t - state[id].tStable >= config[id].gInitTime)
    stable                = 1;

  // state entered upon initialization or lost stability 
  if (state[id].state == IMU_pnts_enum_reset) {
    state[id].state       = IMU_pnts_enum_unstable;
    state[id].tStable     = t;
    entry->tStart         = t;
    entry->gAccum[0]      = g[0];
    entry->gAccum[1]      = g[1];
    entry->gAccum[2]      = g[2];
  } 

  // state occurs prior to reaching "stable" 
  else if (state[id].state == IMU_pnts_enum_unstable) {
    if (!stable) {
      entry->gAccum[0]   += g[0];
      entry->gAccum[1]   += g[1];
      entry->gAccum[2]   += g[2];
      entry->tEnd         = t;
    } else {
      state[id].state     = IMU_pnts_enum_stable;
      state[id].aClock    = 1;
      state[id].mClock    = 1;
      copy_val(entry->gFltr, g);
    }
  }

  // state occurs after reaching "stable" 
  else if (state[id].state == IMU_pnts_enum_stable) {
    if (stable) 
      apply_alpha(entry->gFltr, g, config[id].gAlpha);
    else
      *pntr = break_stable(id, t, tStable);
  } 

  // exit function
  return state[id].curPnts;
}


/******************************************************************************
* process accelerometer vector
******************************************************************************/

int IMU_pnts_newAccl(
  uint16_t                id, 
  IMU_TYPE                t, 
  IMU_TYPE                *a,
  IMU_pnts_entry          **pntr)
{
  // initialize pointer
  *pntr                   = NULL;

  // verify system state (stable and sensor is enabled)  
  if (!config[id].isAccl || state[id].state != IMU_pnts_enum_stable)
    return state[id].curPnts;

  // define local variables
  IMU_pnts_entry *entry   = &table[id][state[id].index];

  // clock sensor data
  if (state[id].aClock) {
    copy_val(entry->aFltr, a);
    state[id].aClock      = 0;
    return state[id].curPnts;
  }

  // stability loss check
  float std = calc_std(entry->aFltr, a);
  if (std > config[id].aThresh * config[id].aThresh) {
    *pntr = break_stable(id, t, state[id].tStable);
    return state[id].curPnts;
  }

  // update points entry
  apply_alpha(entry->aFltr, a, config[id].aAlpha);

  // exit function
  return state[id].curPnts;
}


/******************************************************************************
* process magnetometer vector
******************************************************************************/

int IMU_pnts_newMagn(
  uint16_t                id, 
  IMU_TYPE                t, 
  IMU_TYPE                *m, 
  IMU_pnts_entry          **pntr)
{
  // initialize entry pointer to NULL
  *pntr                   = NULL;

  // verify system state (stable and sensor is enabled)
  if (!config[id].isMagn || state[id].state != IMU_pnts_enum_stable)
    return state[id].curPnts;

  // define local variables
  IMU_pnts_entry *entry   = &table[id][state[id].index];

  // clock sensor data
  if (state[id].mClock) {
    copy_val(entry->mFltr, m);
    state[id].mClock      = 0;
    return state[id].curPnts;
  }

  // stability loss check
  float std = calc_std(entry->mFltr, m);
  if (std > config[id].mThresh * config[id].mThresh) {
    *pntr = break_stable(id, t, state[id].tStable);
    return state[id].curPnts;
  }

  // update points entry
  apply_alpha(entry->mFltr, m, config[id].mAlpha);

  // exit function
  return state[id].curPnts;
}


/******************************************************************************
* utility function - break stable
******************************************************************************/

inline IMU_pnts_entry* break_stable(
  uint16_t                id,
  IMU_TYPE                t,
  IMU_TYPE                tStable)
{
  // reset state machine provided min hold time is not met
  if (t - tStable < config[id].gInitTime + config[id].gHoldTime) {
    state[id].state       = IMU_pnts_enum_reset;
    return NULL;
  }

  // pass pointer to update entry and update state machine
  IMU_pnts_entry *entry   = &table[id][state[id].index];
  state[id].curPnts++;
  state[id].index++;
  if (state[id].index >= IMU_PNTS_SIZE)
    state[id].index       = 0;
  if (state[id].numPnts != 0 && state[id].curPnts >= state[id].numPnts)
    state[id].state       = IMU_pnts_enum_stop;
  else
    state[id].state       = IMU_pnts_enum_reset; 
  return entry;
}


/******************************************************************************
* utility function - apply alpha 
******************************************************************************/

inline float calc_std(
  float                   *val1, 
  IMU_TYPE                *val2)
{
  float diff              = val1[0] - (float)val2[0];  
  float std               = diff * diff;
  diff                    = val1[1] - (float)val2[1];  
  std                    += diff * diff;
  diff                    = val1[2] - (float)val2[2];  
  std                    += diff * diff;
  return std;

}


/******************************************************************************
* utility function - apply alpha 
******************************************************************************/

inline void apply_alpha(
  float                   *prev, 
  IMU_TYPE                *cur, 
  float                   alpha)
{
  prev[0]                 = alpha * (float)cur[0] + (1-alpha) * prev[0];
  prev[1]                 = alpha * (float)cur[1] + (1-alpha) * prev[1]; 
  prev[2]                 = alpha * (float)cur[2] + (1-alpha) * prev[2];

}


/******************************************************************************
* utility function - apply alpha 
******************************************************************************/

inline void copy_val(
  float                   *val1, 
  IMU_TYPE                *val2)
{
  val1[0]                 = val2[0];
  val1[1]                 = val2[1]; 
  val1[2]                 = val2[2];

}

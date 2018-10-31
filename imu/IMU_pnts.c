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

// definitions (increase readability)
#define NULL 0

// include statements 
#include <string.h>
#include "IMU_pnts.h"

// internally managed structures
static IMU_pnts_config  config [IMU_MAX_INST]; 
static IMU_pnts_state   state  [IMU_MAX_INST];
static IMU_pnts_entry   table  [IMU_MAX_INST][IMU_PNTS_SIZE];
static uint16_t         numInst = 0;

// internally defined functions
static inline IMU_pnts_enum   update_state (uint16_t id, uint32_t, uint8_t,
                                            IMU_pnts_entry**);
static inline IMU_pnts_entry* break_stable (uint16_t id);
static inline float calc_std    (float *val1, IMU_TYPE *val2);
static inline void  apply_alpha (float *prev, IMU_TYPE *cur, float alpha);
static inline void  accum_gyro  (float *prev, IMU_TYPE *cur, IMU_TYPE t);
static inline void  copy_val    (float *val1, IMU_TYPE *val2);


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

  // initialize the callback fnc
  state[*id].fnc          = NULL;
  state[*id].fncPntr      = NULL;
  
  // initialize instance state
  IMU_pnts_reset(*id);
  
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
  if (index > state[id].curPnts || index > IMU_PNTS_SIZE-1)
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
* function to reset instance state
******************************************************************************/

// function callback functions
int IMU_pnts_setFnc(
  uint16_t                id, 
  int                     (*fnc)(IMU_PNTS_FNC_ARG),
  void                    *fncPntr)
{
  // check device count overflow
  if (id >= numInst)
    return IMU_PNTS_BAD_INST;

  // copy callback function and its pointer
  state[id].fnc           = fnc;
  state[id].fncPntr       = fncPntr;

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
  state[id].numPnts       = 0;
  state[id].curPnts       = 0;
  state[id].index         = 0;
  state[id].tClock        = 1;
  state[id].gClock        = 1;
  state[id].aClock        = 1;
  state[id].mClock        = 1;
  state[id].current       = &table[id][0];

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
  state[id].state         = IMU_pnts_enum_move;
  state[id].numPnts       = numPnts;
  state[id].curPnts       = 0;
  state[id].tClock        = 1;

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
  state[id].state         = IMU_pnts_enum_move;
  state[id].numPnts       = 0;
  state[id].curPnts       = 0;
  state[id].tClock        = 1;

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
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_PNTS_BAD_INST; 

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
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_PNTS_BAD_INST; 

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
  uint32_t                t,
  IMU_TYPE                *g,
  IMU_pnts_entry          **pntr)
{
  // initialize output to NULL
  *pntr                   = NULL;

  // determine whether the function needs to be executed
  if (!config[id].isGyro || !config[id].enable)
    return IMU_PNTS_FNC_DISABLED;

  // define internal variables
  IMU_pnts_entry *entry   = state[id].current;

  // initialize sensor filter state
  if (state[id].tClock || state[id].gClock) {
    if (state[id].tClock)
      state[id].tStable   = t;
    if (state[id].gClock) {
      copy_val(entry->gFltr, g);
      memset(entry->gAccum, 0, 3*sizeof(float));
      entry->tStart       = t;
      entry->tEnd         = t;
    }
    state[id].tClock      = 0;
    state[id].gClock      = 0;
    state[id].state       = IMU_pnts_enum_move;
    return (int)IMU_pnts_enum_move;
  }

  // calculate sensor mean and deviation
  float std = calc_std(entry->gFltr, g);
  apply_alpha(entry->gFltr, g, config[id].gAlpha);

  // update state based on std and elapsed time
  uint8_t isMove  = std > config[id].gThresh;
  state[id].state = update_state(id, t, isMove, pntr);

  // update accum and counts based on new state
  if (state[id].state == IMU_pnts_enum_move) {
    accum_gyro(entry->gAccum, g, t - entry->tEnd);
    entry->tEnd         = t;
  } else if (state[id].state == IMU_pnts_enum_stable)
    entry->gCount++;
  
  // exit function (pass state)
  return (int)state[id].state;
}


/******************************************************************************
* process accelerometer vector
******************************************************************************/

int IMU_pnts_newAccl(
  uint16_t                id, 
  uint32_t                t, 
  IMU_TYPE                *a,
  IMU_pnts_entry          **pntr)
{
  // initialize output to NULL
  *pntr                   = NULL;

  // define internal variables
  IMU_pnts_entry *entry   = state[id].current;

  // initialize sensor filter state
  if (state[id].tClock || state[id].aClock) {
    if (state[id].tClock)
      state[id].tStable   = t;
    if (state[id].aClock)
      copy_val(entry->aFltr, a);
    state[id].tClock      = 0;
    state[id].aClock      = 0;
    state[id].state       = IMU_pnts_enum_move;
    return (int)IMU_pnts_enum_move;
  }

  // calculate sensor mean and deviation
  float std = calc_std(entry->aFltr, a);
  apply_alpha(entry->aFltr, a, config[id].aAlpha);

  // update state based on std and elapsed time
  uint8_t isMove  = std > config[id].aThresh;
  state[id].state = update_state(id, t, isMove, pntr);

  // update counts based on new state
  if (state[id].state == IMU_pnts_enum_stable)
    entry->aCount++;
  
  // exit function (pass state)
  return (int)state[id].state;
}


/******************************************************************************
* process magnetometer vector
******************************************************************************/

int IMU_pnts_newMagn(
  uint16_t                id, 
  uint32_t                t, 
  IMU_TYPE                *m, 
  IMU_pnts_entry          **pntr)
{
  // initialize output to NULL
  *pntr                   = NULL;

  // define internal variables
  IMU_pnts_entry *entry   = state[id].current;

  // initialize sensor filter state
  if (state[id].tClock || state[id].mClock) {
    if (state[id].tClock)
      state[id].tStable   = t;
    if (state[id].mClock)
      copy_val(entry->mFltr, m);
    state[id].tClock      = 0;
    state[id].mClock      = 0;
    state[id].state       = IMU_pnts_enum_move;
    return (int)IMU_pnts_enum_move;
  }

  // calculate sensor mean and deviation
  float std = calc_std(entry->mFltr, m);
  apply_alpha(entry->mFltr, m, config[id].mAlpha);

  // update state based on std and elapsed time
  uint8_t isMove  = std > config[id].mThresh;
  state[id].state = update_state(id, t, isMove, pntr);

  // update counts based on new state
  if (state[id].state == IMU_pnts_enum_stable)
    entry->mCount++;
  
  // exit function (pass state)
  return (int)state[id].state;
}


/******************************************************************************
* utility function - update state
******************************************************************************/

inline IMU_pnts_enum update_state(
  uint16_t                id,
  uint32_t                t,
  uint8_t                 isMove,
  IMU_pnts_entry          **pntr)
{
  // clear tStable and break_stable given moving
  if (isMove) {
    state[id].tStable     = t;
    if (state[id].state == IMU_pnts_enum_stable)
      *pntr               = break_stable(id);
    return IMU_pnts_enum_move;
  }

  // update state based on elapsed stable time
  if (t - state[id].tStable >= (uint32_t)config[id].tStable)
    return IMU_pnts_enum_stable;
  if (t - state[id].tStable >= (uint32_t)config[id].tHold)
    return IMU_pnts_enum_hold;
  else
    return IMU_pnts_enum_move;
}


/******************************************************************************
* utility function - break stable
******************************************************************************/

inline IMU_pnts_entry* break_stable(
  uint16_t                id)
{
  // update table index and current pointer
  IMU_pnts_entry *entry   = state[id].current;
  state[id].index++;
  if (state[id].index >= IMU_PNTS_SIZE)
    state[id].index       = 0;
  state[id].current       = &table[id][state[id].index];

  // update current points count and launch thread
  if (state[id].curPnts < state[id].numPnts) {
    state[id].curPnts++;
    if (state[id].fnc != NULL)
      state[id].fnc(id, state[id].curPnts-1, entry, state[id].fncPntr);
    return entry;
  } else {
    return NULL;
  }
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
* utility function - apply accum 
******************************************************************************/

inline void accum_gyro(
  float                   *prev, 
  IMU_TYPE                *cur, 
  IMU_TYPE                time)
{
  float time_s            = (float)time * IMU_PNTS_10USEC_TO_SEC;
  prev[0]                += time_s * (float)cur[0];
  prev[1]                += time_s * (float)cur[1];
  prev[2]                += time_s * (float)cur[2];

}


/******************************************************************************
* utility function - apply alpha 
******************************************************************************/

inline void copy_val(
  float                   *val1, 
  IMU_TYPE                *val2)
{
  val1[0]                 = (float)val2[0];
  val1[1]                 = (float)val2[1]; 
  val1[2]                 = (float)val2[2];

}

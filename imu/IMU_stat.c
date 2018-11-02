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
#include <math.h>
#include "IMU_stat.h"

// internally managed structures
static IMU_stat_config    config [IMU_MAX_INST]; 
static IMU_stat_state     state  [IMU_MAX_INST];
static uint16_t           numInst = 0;

// internal functions definitions
int IMU_stat_gyro (uint16_t id, uint32_t t, IMU_TYPE *g, IMU_core_FOM*);
int IMU_stat_accl (uint16_t id, uint32_t t, IMU_TYPE *a, IMU_core_FOM*);
int IMU_stat_magn (uint16_t id, uint32_t t, IMU_TYPE *m, IMU_core_FOM*);


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
* initialize instance to known state
******************************************************************************/

int IMU_stat_reset(
  uint16_t                id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_STAT_BAD_INST; 

  // initialize instantce
  state[id].gBias[0]      = 0.0f;
  state[id].gBias[1]      = 0.0f;
  state[id].gBias[2]      = 0.0f;
  state[id].gBiasStd[0]   = 0.0f;
  state[id].gBiasStd[1]   = 0.0f;
  state[id].gBiasStd[2]   = 0.0f;
  state[id].aMag          = 0.0f;
  state[id].aMagStd       = 0.0f;
  state[id].mMag          = 0.0f;
  state[id].mMagStd       = 0.0f;
  state[id].mDot          = 0.0f;
  state[id].mDotStd       = 0.0f;
  state[id].gClock        = 1;
  state[id].aClock        = 1;
  state[id].mClock        = 1;
  state[id].tGyro         = 0;
  state[id].tAccl         = 0;
  state[id].tMagn         = 0;

  // exit function (no errors)
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
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_STAT_BAD_INST; 
  if (!config[id].enable)
    return IMU_STAT_FNC_DISABLED;

  // check sensor type and execute
  if      (datum->type == IMU_gyro)
    IMU_stat_gyro(id, datum->t, datum->val, FOM);
  else if (datum->type == IMU_accl)
    IMU_stat_accl(id, datum->t, datum->val, FOM);
  else if (datum->type == IMU_magn)
    IMU_stat_magn(id, datum->t, datum->val, FOM);

  // exit function (no errors)
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
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_STAT_BAD_INST; 
  if (!config[id].enable)
    return IMU_STAT_FNC_DISABLED;

  // check sensor type and execute
  IMU_stat_gyro(id, data3->t, data3->g, &FOM[0]);
  IMU_stat_accl(id, data3->t, data3->a, &FOM[1]);
  IMU_stat_magn(id, data3->t, data3->m, &FOM[2]);

  // exit function (no errors)
  return 0;
}


/******************************************************************************
* collect gyroscope statistics
******************************************************************************/

int IMU_stat_gyro(
  uint16_t                id, 
  uint32_t                t, 
  IMU_TYPE                *g,
  IMU_core_FOM            *FOM)
{
  // first sensor datum
  if (state[id].gClock) {
    state[id].gBias[0]    = g[0];
    state[id].gBias[1]    = g[1];
    state[id].gBias[2]    = g[2];
    state[id].gClock      = 0;

  // nominal condition
  } else {
    float dt              = (t-state[id].tGyro) * IMU_STAT_10USEC_TO_SEC;
    float alpha           = dt * config[id].alpha;
    float *gBias          = state[id].gBias;
    float *prev           = state[id].gBiasStd;
    state[id].gBiasStd[0] = (1.0f-alpha)*prev[0] + alpha*fabs(gBias[0]-g[0]); 
    state[id].gBiasStd[1] = (1.0f-alpha)*prev[1] + alpha*fabs(gBias[1]-g[1]); 
    state[id].gBiasStd[2] = (1.0f-alpha)*prev[2] + alpha*fabs(gBias[2]-g[2]); 
    state[id].gBias[0]    = (1.0f-alpha)*gBias[0] + alpha*g[0];
    state[id].gBias[1]    = (1.0f-alpha)*gBias[1] + alpha*g[1];
    state[id].gBias[2]    = (1.0f-alpha)*gBias[2] + alpha*g[2];
  }

  // exit function (no errors)
  state[id].tGyro       = t;
  return 0;
}


/******************************************************************************
* collect accelerometer statistics
******************************************************************************/

int IMU_stat_accl(
  uint16_t                id, 
  uint32_t                t, 
  IMU_TYPE                *a,
  IMU_core_FOM            *FOM)
{
  // first sensor datum
  if (state[id].gClock) {
    state[id].aMag        = FOM->FOM.accl.mag;
    state[id].aMagFOM     = FOM->FOM.accl.magFOM;
    state[id].gClock      = 0;

  // nominal condition
  } else {
    float dt              = (t-state[id].tGyro) * IMU_STAT_10USEC_TO_SEC;
    float alpha           = dt * config[id].alpha;
    float prevMag         = state[id].aMag;
    float prevStd         = state[id].aMagStd;
    float prevFOM         = state[id].aMagFOM;
    float curMag          = FOM->FOM.accl.mag;
    float curFOM          = FOM->FOM.accl.magFOM;
    state[id].aMagStd     = (1.0f-alpha)*prevStd + alpha*fabs(prevMag-curMag);
    state[id].aMag        = (1.0f-alpha)*prevMag + alpha*curMag;
    state[id].aMagFOM     = (1.0f-alpha)*prevFOM + alpha*curFOM;
  }

  // exit function (no errors)
  state[id].tGyro       = t;
  return 0;
}


/******************************************************************************
* collect accelerometer statistics
******************************************************************************/

int IMU_stat_magn(
  uint16_t                id, 
  uint32_t                t, 
  IMU_TYPE                *m,
  IMU_core_FOM            *FOM)
{
  // first sensor datum
  if (state[id].mClock) {
    state[id].mMag        = FOM->FOM.magn.mag;
    state[id].mMagFOM     = FOM->FOM.magn.magFOM;
    state[id].mDot        = FOM->FOM.magn.dot;
    state[id].mDotFOM     = FOM->FOM.magn.dotFOM;
    state[id].mClock      = 0;

  // nominal condition
  } else {
    float dt              = (t-state[id].tMagn) * IMU_STAT_10USEC_TO_SEC;
    float alpha           = dt * config[id].alpha;
    float prevMag         = state[id].mMag;
    float prevStd         = state[id].mMagStd;
    float prevFOM         = state[id].mMagFOM;
    float curMag          = FOM->FOM.magn.mag;
    float curFOM          = FOM->FOM.magn.magFOM;
    state[id].mMagStd     = (1.0f-alpha)*prevStd + alpha*fabs(prevMag-curMag);
    state[id].mMag        = (1.0f-alpha)*prevMag + alpha*curMag;
    state[id].mMagFOM     = (1.0f-alpha)*prevFOM + alpha*curFOM;
  }

  // exit function (no errors)
  state[id].tGyro       = t;
  return 0;
}

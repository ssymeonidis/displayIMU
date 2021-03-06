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
#include <math.h>             // sqrt
#include <string.h>           // memcpy
#if IMU_USE_PTHREAD
#include <pthread.h>
#include <unistd.h>
#endif
#include "IMU_calb.h"

// internally defined variables
static IMU_calb_config  config [IMU_MAX_INST];
static IMU_calb_state   state  [IMU_MAX_INST];
static IMU_pnts_entry   table  [IMU_MAX_INST][IMU_CALB_SIZE]; 
static uint16_t         numInst  = 0;
#if IMU_USE_PTHREAD
static pthread_t        thrd     [IMU_MAX_INST];
static pthread_attr_t   thrdAttr [IMU_MAX_INST];
static uint16_t         thrdVal  [IMU_MAX_INST];
#endif

// internally defined functions
static void calb_1pnt_gyro (uint16_t id);
static void calb_4pnt_magn (uint16_t id);
static void calb_6pnt_full (uint16_t id);
void* IMU_calb_update     (void    *id);
void  IMU_calb_defaultFnc (IMU_CALB_FNC_ARG);


/******************************************************************************
* initialize new instance (constructor) 
******************************************************************************/

int IMU_calb_init(
  uint16_t                *id,
  IMU_calb_config         **pntr)
{
  // check device count overflow
  if (numInst >= IMU_MAX_INST)
    return IMU_CALB_INST_OVERFLOW;

  // initialize to known state
  config[numInst].enable  = 1;

  // assign internal calb function
  state[numInst].fnc      = IMU_calb_defaultFnc;
  state[numInst].fncPntr  = NULL;
  
  // pass handle and config pointer
  *id                     = numInst; 
  *pntr                   = &config[*id];
  numInst++;
  
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* return config structure
******************************************************************************/

int IMU_calb_getConfig( 
  uint16_t                id,  
  IMU_calb_config         **pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CALB_BAD_INST; 

  // pass config and exit (no errors)
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* copies rect and core subsystem configuration structures
******************************************************************************/

int IMU_calb_setStruct(
  uint16_t                id,
  IMU_rect_config         *rect, 
  IMU_core_config         *core)
{
  // check device count overflow
  if (id >= numInst)
    return IMU_CALB_BAD_INST;

  // copy rectify and core structures
  state[id].rectPntr      = rect;
  state[id].corePntr      = core;

  // exit function (no errors)
  return 0;
}


/******************************************************************************
* copies rect and core subsystem configuration structures
******************************************************************************/

int IMU_calb_setFnc(
  uint16_t                id,
  void                    (*fnc)(IMU_CALB_FNC_ARG),
  void                    *fncPntr)
{
  // check device count overflow
  if (fnc == NULL)
    return IMU_CALB_BAD_PNTR;

  // copy rectify and core structures
  state[id].fnc           = fnc;
  state[id].fncPntr       = fncPntr;
  
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* reset states of calb subsystem
******************************************************************************/

int IMU_calb_reset(
  uint16_t                id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CALB_BAD_INST;

  // copy current entry to the table
  state[id].numPnts       = 0;
  
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* starts calibration
******************************************************************************/

int IMU_calb_start(
  uint16_t                id,
  IMU_calb_mode           mode,
  void                    *pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CALB_BAD_INST;
  if (!config[id].enable)
    return IMU_CALB_FNC_DISABLED;

  // copy current entry to the table
  state[id].numPnts       = 0;
  state[id].mode          = mode;
  state[id].calbArg       = pntr;
  memcpy(&state[id].rect, state[id].rectPntr, sizeof(IMU_rect_config));
  memcpy(&state[id].core, state[id].corePntr, sizeof(IMU_core_config));
  
  // exit function (no errors)
  return IMU_calb_mode_pnts[mode];
}



/******************************************************************************
* update IMU_correct structure w/ statcal results
******************************************************************************/

int IMU_calb_stat(
  uint16_t                id,
  IMU_stat_state          *stat)
{
  // copy current entry to the table
  memcpy(&state[id].rect, state[id].rectPntr, sizeof(IMU_rect_config));
  memcpy(&state[id].core, state[id].corePntr, sizeof(IMU_core_config));
  
  // update core config
  IMU_core_config *core   = &state[id].core;
  float sigma             = config[id].sigma;
  core->aMag              = stat->aMag;
  core->mMag              = stat->mMag;
  core->mDot              = stat->mDot;
  core->aMagThresh        = stat->aMagStd * sigma;
  core->mMagThresh        = stat->mMagStd * sigma;
  core->mDotThresh        = stat->mDotStd * sigma;

  // exit function (no errors)
  return 0;
}


/******************************************************************************
* save results to rect and core configuation structures
******************************************************************************/

int IMU_calb_save(
  uint16_t                id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CALB_BAD_INST;
  if (!config[id].enable)
    return IMU_CALB_FNC_DISABLED;

  // create temporary swap storage
  IMU_rect_config          rect;
  IMU_core_config          core;

  // copy current entry to the IMU rectify config 
  memcpy(&rect, &state[id].rect, sizeof(IMU_rect_config));
  memcpy(&core, &state[id].core, sizeof(IMU_core_config));
  memcpy(&state[id].rect, state[id].rectPntr, sizeof(IMU_rect_config));
  memcpy(&state[id].core, state[id].corePntr, sizeof(IMU_core_config));
  memcpy(state[id].rectPntr, &rect, sizeof(IMU_rect_config));
  memcpy(state[id].corePntr, &core, sizeof(IMU_core_config));
  
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* save results to rect and core configuation structures
******************************************************************************/

int IMU_calb_revert(
  uint16_t                id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CALB_BAD_INST;
  if (!config[id].enable)
    return IMU_CALB_FNC_DISABLED;

  // copy current entry to the IMU rectify config 
  memcpy(state[id].rectPntr, &state[id].rect, sizeof(IMU_rect_config));
  memcpy(state[id].corePntr, &state[id].core, sizeof(IMU_core_config));
  
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* adds points for calibration
******************************************************************************/

int IMU_calb_point(
  uint16_t                id, 
  IMU_pnts_entry          *pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CALB_BAD_INST; 
  if (!config[id].enable)
    return IMU_CALB_FNC_DISABLED;

  // copy current entry to the table
  IMU_pnts_entry *entry = &table[id][state[id].numPnts]; 
  memcpy(entry, pntr, sizeof(IMU_pnts_entry));
  state[id].numPnts++;

  // determine whether enough points were collected
  if (state[id].numPnts >= IMU_calb_mode_pnts[state[id].mode]) {
    thrdVal[id] = id;
    #if IMU_USE_PTHREAD
    pthread_create(&thrd[id], &thrdAttr[id], IMU_calb_update, &thrdVal);
    #else
    IMU_calb_update(&id);
    #endif    
    return IMU_CALB_UPDATED;
  }
    
  // exit fuction (no errors)
  return 0;
}

/******************************************************************************
* adds points for calibration
******************************************************************************/

void* IMU_calb_update(
  void                    *pntr)
{
  // define internal variables
  int *id                 = (int*)pntr;  

  // perform the specified calibration routine
  if      (state[*id].mode == IMU_calb_1pnt_gyro)
    calb_1pnt_gyro(*id);
  else if (state[*id].mode == IMU_calb_4pnt_magn)
    calb_4pnt_magn(*id);
  else if (state[*id].mode == IMU_calb_6pnt_full)
    calb_6pnt_full(*id);

  // call calibration function
  state[*id].fnc(*id, &state[*id].FOM, state[*id].fncPntr);

  // exit function (no errors)
  return NULL;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

void calb_1pnt_gyro(
  uint16_t                id)
{
  // process completed points table
  IMU_rect_config *rect   = &state[id].rect;
  rect->gBias[0]          = -table[id][0].gFltr[0];
  rect->gBias[1]          = -table[id][0].gFltr[1];
  rect->gBias[2]          = -table[id][0].gFltr[2];
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

void calb_4pnt_magn(
  uint16_t                id)
{
  // define internal variables
  IMU_rect_config *rect   = &state[id].rect;
  IMU_core_config *core   = &state[id].core;
  float                   *g;
  float                   *m;
  float                   *a;
  int                     i;

  // process completed points table
  rect->gBias[0]          = 0.0f;
  rect->gBias[1]          = 0.0f;
  rect->gBias[2]          = 0.0f;
  rect->mBias[0]          = 0.0f;
  rect->mBias[1]          = 0.0f;
  core->mMag              = 0.0f;
  core->mDot              = 0.0f;
  for (i=0; i<4; i++) {
    g                     = table[id][i].gFltr;
    a                     = table[id][i].aFltr;
    m                     = table[id][i].mFltr;
    rect->gBias[0]       -= g[0];
    rect->gBias[1]       -= g[1];
    rect->gBias[2]       -= g[2];
    rect->mBias[0]       -= m[0];
    rect->mBias[1]       -= m[1];
    core->mMag           += sqrt(m[0]*m[0] + m[1]*m[1] + m[2]*m[2]);
    core->mDot           += a[0]*m[0] + a[1]*m[1] + a[2]*m[2];
  }

  // convert sums to averages
  rect->gBias[0]         /= 4.0f;
  rect->gBias[1]         /= 4.0f;
  rect->gBias[2]         /= 4.0f;
  rect->mBias[0]         /= 4.0f;
  rect->mBias[1]         /= 4.0f;
  core->mMag             /= 4.0f;
  core->mDot             /= 4.0f;
}


/******************************************************************************
* function to return instance state pointer
******************************************************************************/

void calb_6pnt_full(
  uint16_t                id)
{ 
  // define internal variables
  IMU_rect_config *rect   = &state[id].rect;
  IMU_core_config *core   = &state[id].core;
  float                   *g;
  float                   *a;
  int                     i;

  // process completed points table
  rect->gBias[0]          = 0.0f;
  rect->gBias[1]          = 0.0f;
  rect->gBias[2]          = 0.0f;
  rect->aBias[0]          = 0.0f;
  rect->aBias[1]          = 0.0f;
  rect->aBias[2]          = 0.0f;
  core->aMag              = 0.0f;
  for (i=0; i<6; i++) {
    g                     = table[id][i].gFltr;
    a                     = table[id][i].aFltr;
    rect->gBias[0]       -= g[0];
    rect->gBias[1]       -= g[1];
    rect->gBias[2]       -= g[2];
    rect->aBias[0]       -= a[0];
    rect->aBias[1]       -= a[1];
    rect->aBias[2]       -= a[2];
    core->aMag           += sqrt(a[0]*a[0] + a[1]*a[1] + a[2]*a[2]);
  }

  // convert sums to averages
  rect->gBias[0]         /= 6.0f;
  rect->gBias[1]         /= 6.0f;
  rect->gBias[2]         /= 6.0f;
  rect->aBias[0]         /= 6.0f;
  rect->aBias[1]         /= 6.0f;
  rect->aBias[2]         /= 6.0f;
  core->aMag             /= 6.0f;
}


/******************************************************************************
* function to estimate orientation and acceleration
******************************************************************************/

void IMU_calb_defaultFnc(
  uint16_t                id,
  IMU_calb_FOM            *FOM,
  void                    *pntr)
{
  // check out-of-bounds condition
  if (FOM == NULL)
    return;
  if (pntr != NULL)
    return;
  
  // verify FOM is above threshold and save
  IMU_calb_save(id);
}

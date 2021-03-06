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
#define max(a,b)              \
  ({__typeof__ (a) _a = (a);  \
    __typeof__ (b) _b = (b);  \
    _a > _b ? _a : _b; })

// include statements 
#if IMU_USE_PTHREAD
#include <pthread.h>
#include <unistd.h>
#endif
#include <string.h>
#include "IMU_file.h"
#include "IMU_thrd.h"
#include "IMU_math.h"
#include "IMU_engn.h"

// internally defined types
#if IMU_ENGN_QUEUE_SIZE
typedef struct {
  uint16_t               id       [IMU_ENGN_QUEUE_SIZE+1];
  IMU_datum              datum    [IMU_ENGN_QUEUE_SIZE+1];
  int                    first;
  int                    last;
  int                    count;
} IMU_engn_queue;
#endif

// internally define variables
static IMU_core_FOM      datumFOM [3];
static IMU_engn_config   config   [IMU_MAX_INST]; 
static IMU_engn_state    state    [IMU_MAX_INST];
static IMU_engn_sensor   sensor   [IMU_MAX_INST];
static uint16_t          numInst = 0;
#if IMU_ENGN_QUEUE_SIZE
static IMU_engn_queue    queue;
#endif
#if IMU_USE_PTHREAD
static useconds_t        sleepTime = 20;
static pthread_mutex_t   thrdLock;
static pthread_t         thrd;
static pthread_attr_t    thrdAttr;
static uint8_t           thrdExit;
static uint8_t           thrdIsExit;
#endif

// internally defined functions
int IMU_engn_calbFnc    (uint16_t id, IMU_calb_FOM*);
int IMU_engn_process    (uint16_t id, IMU_datum*);
int IMU_copy_datumRaw   (uint16_t id, IMU_datum*);
int IMU_copy_data3Raw   (uint16_t id, IMU_data3*);
int IMU_copy_results1   (uint16_t id, IMU_datum*, IMU_core_FOM*);
int IMU_copy_results3   (uint16_t id, IMU_data3*, IMU_core_FOM*);
int IMU_engn_typeCheck  (uint16_t id, IMU_engn_system);
#if IMU_ENGN_QUEUE_SIZE
int IMU_engn_addQueue   (uint16_t id, IMU_datum*);
void* IMU_engn_run      (void*);
#endif


/******************************************************************************
* function for creating new instance
******************************************************************************/

int IMU_engn_init(
  IMU_engn_type         type,
  uint16_t              *id)
{
  // check device count overflow
  if (numInst >= IMU_MAX_INST)
    return IMU_ENGN_INST_OVERFLOW;

  // pass handle and config structure
  *id                   = numInst;
  numInst++;

  // initialize config structure
  if        (type == IMU_engn_core_only) {
    config[*id].isRect       = 0;
    config[*id].isPnts       = 0;
    config[*id].isStat       = 0;
    config[*id].isCalb       = 0;
  } else if (type == IMU_engn_rect_core) {
    config[*id].isRect       = 1;
    config[*id].isPnts       = 0;
    config[*id].isStat       = 0;
    config[*id].isCalb       = 0;
  } else if (type == IMU_engn_calb_pnts) {
    config[*id].isRect       = 1;
    config[*id].isPnts       = 1;
    config[*id].isStat       = 0;
    config[*id].isCalb       = 1;
  } else if (type == IMU_engn_calb_stat) {
    config[*id].isRect       = 1;
    config[*id].isPnts       = 0;
    config[*id].isStat       = 1;
    config[*id].isCalb       = 1;
  } else if (type == IMU_engn_calb_full) {
    config[*id].isRect       = 1;
    config[*id].isPnts       = 1;
    config[*id].isStat       = 1;
    config[*id].isCalb       = 1;
  } else {
    return IMU_ENGN_BAD_ENGN_TYPE;
  }
  config[*id].isFOM          = 0;
  config[*id].isTran         = 0;
  config[*id].isRef          = 1;
  config[*id].isAng          = 1;
  config[*id].isSensorStruct = 0;
  config[*id].qRef[0]        = 1;
  config[*id].qRef[1]        = 0;
  config[*id].qRef[2]        = 0;
  config[*id].qRef[3]        = 0;
  state[*id].rect            = 0;
  state[*id].pnts            = 0;
  state[*id].stat            = 0;
  state[*id].calb            = 0;
  state[*id].datumCount      = 0;
  
  // create IMU subsystem instances
  IMU_engn_state *cur = &state[*id];
  state[*id].core     = IMU_core_init(&cur->idCore, &cur->configCore);
  if (config[*id].isRect)
    state[*id].rect   = IMU_rect_init(&cur->idRect, &cur->configRect);
  if (config[*id].isPnts)
    state[*id].pnts   = IMU_pnts_init(&cur->idPnts, &cur->configPnts);
  if (config[*id].isStat)
    state[*id].stat   = IMU_stat_init(&cur->idStat, &cur->configStat);
  if (config[*id].isCalb)
    state[*id].calb   = IMU_calb_init(&cur->idCalb, &cur->configCalb);
  if (cur->core < 0 || cur->rect < 0 || cur->pnts < 0 ||
      cur->stat < 0 || cur->calb < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;

  // initialize config structures
  if (config[*id].isCalb > 0)
    IMU_calb_setStruct(cur->idCalb, cur->configRect, cur->configCore);

  // create pthread mutex
  #if IMU_USE_PTHREAD
  int err  = IMU_thrd_mutex_init(&thrdLock);
  if (err) return IMU_CORE_FAILED_MUTEX;
  #endif

  // exit (no errors)
  return 0;
}


/******************************************************************************
* function to return subsystem ID
******************************************************************************/

int IMU_engn_getSysID(
  uint16_t		id,  
  IMU_engn_system       system,
  uint16_t              *sysID)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;

  // pass subsystem handle (id)
  if      (system == IMU_engn_core)
    *sysID              = state[id].idCore;
  else if (system == IMU_engn_rect) 
    *sysID              = state[id].idRect;
  else if (system == IMU_engn_pnts) 
    *sysID              = state[id].idPnts;
  else if (system == IMU_engn_stat) 
    *sysID              = state[id].idStat;
  else if (system == IMU_engn_calb)
    *sysID              = state[id].idCalb;
  else
    return IMU_ENGN_NONEXISTANT_SYSID;
  
  // exit (no errors)
  return 0;
}


/******************************************************************************
* function to return config structure
******************************************************************************/

int IMU_engn_getConfig( 
  uint16_t              id,  
  IMU_engn_system       system, 
  IMU_union_config      *pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST; 

  // pass subsystem config structure
  if      (system == IMU_engn_core)
    pntr->core          = state[id].configCore;
  else if (system == IMU_engn_rect)
    pntr->rect          = state[id].configRect;
  else if (system == IMU_engn_pnts)
    pntr->pnts          = state[id].configPnts;
  else if (system == IMU_engn_stat)
    pntr->stat          = state[id].configStat;
  else if (system == IMU_engn_calb)
    pntr->calb          = state[id].configCalb;
  else if (system == IMU_engn_self)
    pntr->engn          = &config[id];
  else
    return IMU_ENGN_NONEXISTANT_SYSID;
  
  // exit (no errors)
  return 0;
}


/******************************************************************************
* function to return state structure
******************************************************************************/

int IMU_engn_getState( 
  uint16_t		id,  
  IMU_engn_system       system, 
  IMU_union_state       *pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;

  // pass subsystem state structure
  if      (system == IMU_engn_core) 
    IMU_core_getState(state[id].idCore, &pntr->core);
  else if (system == IMU_engn_rect)
    return IMU_ENGN_NONEXISTANT_STRUCT;
  else if (system == IMU_engn_pnts)
    IMU_pnts_getState(state[id].idPnts, &pntr->pnts);
  else if (system == IMU_engn_stat)
    IMU_stat_getState(state[id].idStat, &pntr->stat);
  else if (system == IMU_engn_calb)
    return IMU_ENGN_NONEXISTANT_STRUCT;
  else if (system == IMU_engn_self)
    pntr->engn          = &state[id];
  else
    return IMU_ENGN_NONEXISTANT_SYSID;

  // pass state structure and exit
  return 0;
}


/******************************************************************************
* function to return subsystem ID
******************************************************************************/

int IMU_engn_getSensor( 
  uint16_t		id,  
  IMU_engn_sensor       **pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST; 
  if (!config[id].isSensorStruct)
    return IMU_ENGN_DISABLED_SENSOR_STRUCT;

  // pass sensor structure and exit
  *pntr = &sensor[id];
  return 0;
}


/******************************************************************************
* function to set calibration callback
******************************************************************************/

int IMU_engn_setCalbFnc( 
  uint16_t		id,  
  void                  (*fnc)(IMU_CALB_FNC_ARG),
  void                  *fncPntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;
  if (!config[id].isCalb)
    return IMU_ENGN_UNINITIALIZE_SYS;

  // pass sensor structure and exit
  int status = IMU_calb_setFnc(id, fnc, fncPntr);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;

  // exit function
  return 0;
}


/******************************************************************************
* function to set calibration callback
******************************************************************************/

int IMU_engn_setStableFnc( 
  uint16_t		id,  
  void                  (*fnc)(IMU_PNTS_FNC_ARG),
  void                  *fncPntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;
  if (!config[id].isCalb)
    return IMU_ENGN_UNINITIALIZE_SYS;

  // pass sensor structure and exit
  int status = IMU_pnts_fncStable(id, fnc, fncPntr);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;

  // exit function
  return 0;
}


/******************************************************************************
* function to set calibration callback
******************************************************************************/

int IMU_engn_setBreakFnc( 
  uint16_t		id,  
  void                  (*fnc)(IMU_PNTS_FNC_ARG),
  void                  *fncPntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;
  if (!config[id].isCalb)
    return IMU_ENGN_UNINITIALIZE_SYS;

  // pass sensor structure and exit
  int status = IMU_pnts_fncBreak(id, fnc, fncPntr);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;

  // exit function
  return 0;
}


/******************************************************************************
* function to set calibration callback
******************************************************************************/

int IMU_engn_start()
{
  // reset all instances
  int             status  = 0;
  int             i;
  for (i=0; i<numInst; i++)
    status  += max(0, IMU_engn_reset(i));
  if (status > 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;

  // zero queue size pass through
  #if IMU_ENGN_QUEUE_SIZE == 0
  return 0;
  #endif
  
  // initialize datum queue
  queue.first     = 0;
  queue.last      = IMU_ENGN_QUEUE_SIZE - 1;
  queue.count     = 0;
  
  // currently supports pthread only
  #if IMU_USE_PTHREAD
  thrdExit        = 0;
  return pthread_create(&thrd, &thrdAttr, IMU_engn_run, NULL);
  #else
  return 0;
  #endif
}


/******************************************************************************
* function to stop IMU engn
******************************************************************************/

int IMU_engn_stop()
{
  // zero queue size pass through
  #if IMU_ENGN_QUEUE_SIZE == 0
  return 0;
  #endif
  
  #if IMU_USE_PTHREAD
  thrdExit = 1;
  while (!thrdIsExit)
    usleep(sleepTime);
  pthread_join(thrd, NULL);
  return 0;
  #else
  return IMU_ENGN_FAILED_THREAD;
  #endif
}


/******************************************************************************
* function to load json configuration file
******************************************************************************/

int IMU_engn_load(
  uint16_t              id,
  const char*           filename,
  IMU_engn_system       system)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;
    
  // load respective json file
  if      (system == IMU_engn_core)
    return   IMU_file_coreLoad(filename, state[id].configCore);
  else if (system == IMU_engn_rect)
    return IMU_file_rectLoad(filename, state[id].configRect);
  else if (system == IMU_engn_pnts)
    return IMU_file_pntsLoad(filename, state[id].configPnts);
  else if (system == IMU_engn_stat)
    return IMU_file_statLoad(filename, state[id].configStat);
  else if (system == IMU_engn_calb) 
    return IMU_file_calbLoad(filename, state[id].configCalb);
  else if (system == IMU_engn_self) {
    int status = IMU_file_engnLoad(filename, &config[id]);
    if (config[id].configFileCore[0] != '\0')
      IMU_file_coreLoad(config[id].configFileCore, state[id].configCore);
    if (config[id].configFileRect[0] != '\0')
      IMU_file_rectLoad(config[id].configFileRect, state[id].configRect);
    if (config[id].configFilePnts[0] != '\0')
      IMU_file_pntsLoad(config[id].configFilePnts, state[id].configPnts);
    if (config[id].configFileStat[0] != '\0')
      IMU_file_statLoad(config[id].configFileStat, state[id].configStat);
    if (config[id].configFileCalb[0] != '\0')
      IMU_file_calbLoad(config[id].configFileCalb, state[id].configCalb);
    return status;
  } else {
    return IMU_ENGN_NONEXISTANT_SYSID;
  }
}


/******************************************************************************
* function to save json configuration file
******************************************************************************/

int IMU_engn_save(
  uint16_t              id,
  const char*           filename,
  IMU_engn_system       system)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;
    
  // load respective json file
  if      (system == IMU_engn_core)
    return IMU_file_coreSave(filename, state[id].configCore);
  else if (system == IMU_engn_rect)
    return IMU_file_rectSave(filename, state[id].configRect);
  else if (system == IMU_engn_pnts)
    return IMU_file_pntsSave(filename, state[id].configPnts);
  else if (system == IMU_engn_stat)
    return IMU_file_statSave(filename, state[id].configStat);
  else if (system == IMU_engn_calb)
    return IMU_file_calbSave(filename, state[id].configCalb);
  else if (system == IMU_engn_self) {
    int status = IMU_file_engnSave(filename, &config[id]);
    if (config[id].configFileCore[0] != '\0')
      IMU_file_coreSave(config[id].configFileCore, state[id].configCore);
    if (config[id].configFileRect[0] != '\0')
      IMU_file_rectSave(config[id].configFileRect, state[id].configRect);
    if (config[id].configFilePnts[0] != '\0')
      IMU_file_pntsSave(config[id].configFilePnts, state[id].configPnts);
    if (config[id].configFileStat[0] != '\0')
      IMU_file_statSave(config[id].configFileStat, state[id].configStat);
    if (config[id].configFileCalb[0] != '\0')
      IMU_file_calbSave(config[id].configFileCalb, state[id].configCalb);
    return status;
  } else {
    return IMU_ENGN_NONEXISTANT_SYSID;
  }
}


/******************************************************************************
* function to reset entire imu system
******************************************************************************/

int IMU_engn_reset(
  uint16_t              id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST; 
    
  // initialize sensor structure
  sensor[id].time         = 0;
  memset(sensor[id].gRaw, 0, 3*sizeof(IMU_TYPE));
  memset(sensor[id].gCor, 0, 3*sizeof(IMU_TYPE));
  memset(sensor[id].gFlt, 0, 3*sizeof(IMU_TYPE));
  memset(sensor[id].aRaw, 0, 3*sizeof(IMU_TYPE));
  memset(sensor[id].aCor, 0, 3*sizeof(IMU_TYPE));
  memset(sensor[id].aFlt, 0, 3*sizeof(IMU_TYPE));
  memset(sensor[id].mRaw, 0, 3*sizeof(IMU_TYPE));
  memset(sensor[id].mCor, 0, 3*sizeof(IMU_TYPE));
  memset(sensor[id].mFlt, 0, 3*sizeof(IMU_TYPE));
  
  // reset all open subsystems
  state[id].core   = IMU_core_reset(state[id].idCore);
  if (config[id].isPnts)
    state[id].pnts = IMU_pnts_reset(state[id].idPnts);
  if (config[id].isStat)
    state[id].stat = IMU_stat_reset(state[id].idStat);
  if (config[id].isCalb)
    state[id].calb = IMU_calb_reset(state[id].idCalb);
  if (state[id].core < 0 || state[id].pnts < 0 || 
      state[id].stat < 0 || state[id].calb < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
   
  // exit function
  return 0;
}


/******************************************************************************
* function to set reference quaternion (manual)
******************************************************************************/

int IMU_engn_setRef( 
  uint16_t		id,
  float                 *ref)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;

  // copy contents of input vector
  config[id].qRef[0]    = ref[0];
  config[id].qRef[1]    = ref[1];
  config[id].qRef[2]    = ref[2];
  config[id].qRef[3]    = ref[3];
  
  // exit function
  return 0;
}


/******************************************************************************
* function to set reference quaternion (current orientation)
******************************************************************************/

int IMU_engn_setRefCur( 
  uint16_t		id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;

  // define local variables
  float                 ref[4];
  int                   status;

  // get current orientation and pass to setRef
  status = IMU_core_estmQuat(state[id].idCore, 0, ref);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;

  // copy contents of input vector
  config[id].qRef[0]    = ref[0];
  config[id].qRef[1]    = ref[1];
  config[id].qRef[2]    = ref[2];
  config[id].qRef[3]    = ref[3];
  
  // exit function
  return 0;
}


/******************************************************************************
* function to set reference quaternion (current orientation)
******************************************************************************/

int IMU_engn_calbStart( 
  uint16_t		id,
  IMU_calb_mode         mode,
  void                  *pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;
  
  // get current orientation and pass to setRef
  int status = IMU_calb_start (state[id].idCalb, mode, pntr);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
  status     = IMU_pnts_start (state[id].idPnts, status);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* function to set reference quaternion (current orientation)
******************************************************************************/

int IMU_engn_calbStat( 
  uint16_t		id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;
  
  // get pointer to stat state
  IMU_union_state engn_state;
  int status = IMU_engn_getState(0, IMU_engn_stat, &engn_state); 
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;

  // get current orientation and pass to setRef
  status     = IMU_calb_stat (state[id].idCalb, engn_state.stat);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
  status     = IMU_calb_save (state[id].idCalb);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* function to set reference quaternion (current orientation)
******************************************************************************/

int IMU_engn_calbSave( 
  uint16_t		id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;

  // get current orientation and pass to setRef
  int status = IMU_calb_save(state[id].idCalb);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // exit function
  return 0;
}


/******************************************************************************
* function to set reference quaternion (current orientation)
******************************************************************************/

int IMU_engn_calbRevert( 
  uint16_t		id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;

  // get current orientation and pass to setRef
  int status = IMU_calb_revert(state[id].idCalb);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // exit function
  return 0;
}


/******************************************************************************
* function to receive a datum
******************************************************************************/

int IMU_engn_datum( 
  uint16_t		id,
  IMU_datum             *datum)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;
    
  // non-blocking call will add datum to queue
  #if IMU_ENGN_QUEUE_SIZE
  return IMU_engn_addQueue(id, datum);
  #else
  return IMU_engn_process(id, datum);
  #endif
}


/******************************************************************************
* function to receive data3 (synchronized sensors)
******************************************************************************/

int IMU_engn_data3(
  uint16_t              id, 
  IMU_data3             *data3)
{
  // define local variables
  IMU_core_FOM          *FOM   = NULL;
  IMU_pnts_entry        *pnt   = NULL;
  IMU_pnts_enum         status;
    
  // create FOM pointer
  if (config[id].isStat || config[id].isFOM)
    FOM = datumFOM;

  // save data to sensor structure
  if (config[id].isSensorStruct)
    IMU_copy_data3Raw(id, data3);

  // update the datum counter
  state[id].datumCount++;
  
  // process datum by subsystems
  if (config[id].isRect)
    state[id].rect = IMU_rect_data3(state[id].idRect, data3);
  if (config[id].isPnts) {
    state[id].pnts = IMU_pnts_data3(state[id].idPnts, data3, &pnt);
    status         = (IMU_pnts_enum)state[id].pnts;
  } else {
    status         = IMU_pnts_enum_stable;
  }
  state[id].core   = IMU_core_data3(state[id].idCore, data3, FOM);
  if (config[id].isCalb && pnt != NULL)
    state[id].calb = IMU_calb_point(state[id].idCalb, pnt);
  if (config[id].isStat)
    state[id].stat = IMU_stat_data3(state[id].idStat, data3, FOM, status);
  if (state[id].rect < 0 || state[id].pnts < 0 ||
      state[id].core < 0 || state[id].stat < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // save data to sensor structure
  if (config[id].isSensorStruct)
    IMU_copy_results3(id, data3, FOM);
    
  // exit function
  return 0;
}


/******************************************************************************
* function to estimate orientation and acceleration
******************************************************************************/

int IMU_engn_getEstm(
  uint16_t              id, 
  float                 t,
  IMU_engn_estm         *estm)
{
  // get estimates
  int status = IMU_core_estmQuat(state[id].idCore, t, estm->qOrg);
  if (config[id].isTran)
    IMU_core_estmAccl(state[id].idCore, t, estm->tran);
  if (config[id].isRef)
    IMU_math_quatMultConj(estm->qOrg, config[id].qRef, estm->q);
  if (config[id].isAng &&  config[id].isRef)
    IMU_math_quatToEuler(estm->q, estm->ang);
  if (config[id].isAng && !config[id].isRef)
    IMU_math_quatToEuler(estm->qOrg, estm->ang);
  
  // exit function
  if (status < 0)
    return status;
  else
    return state[id].datumCount;
}


/******************************************************************************
* "continous run" function handle for recreating a thread 
******************************************************************************/

#if IMU_ENGN_QUEUE_SIZE
void* IMU_engn_run(
  void                  *pntr)
{
  // check input argument
  if (pntr != NULL)
    return NULL;

  // define local variables
  uint16_t              id;
  IMU_datum             datum;

  // main processing loop
  thrdIsExit            = 0;
  while(!thrdExit) {
  
    // wait until queue contains datum
    while (queue.count == 0)
      usleep(sleepTime);
    
    // remove datum from queue
    IMU_thrd_mutex_lock(&thrdLock);
    id                  = queue.id[queue.first];
    memcpy(&datum, &queue.datum[queue.first], sizeof(IMU_datum));
    queue.count         = queue.count - 1;
    queue.first         = queue.first + 1;
    if (queue.first >= IMU_ENGN_QUEUE_SIZE)
      queue.first       = 0;
    IMU_thrd_mutex_unlock(&thrdLock);
          
    // process datum
    IMU_engn_process(id, &datum);
  }
  thrdIsExit            = 1;
  return NULL;
}
#endif


/******************************************************************************
* function to receive a datum
******************************************************************************/

#if IMU_ENGN_QUEUE_SIZE
int IMU_engn_addQueue( 
  uint16_t		id,
  IMU_datum             *datum)
{
  // define local variables
  int                   status = 0;

  // entering critical section
  IMU_thrd_mutex_lock(&thrdLock);

  // check queue overflow
  if (queue.count >= IMU_ENGN_QUEUE_SIZE) {
    queue.count         = queue.count - 1;
    queue.first         = queue.first + 1;
    if (queue.first >= IMU_ENGN_QUEUE_SIZE)
      queue.first       = 0; 
    status = IMU_ENGN_QUEUE_OVERFLOW;
  }
  
  // add datum to queue
  queue.count           = queue.count + 1;
  queue.last            = queue.last  + 1;
  if (queue.last >= IMU_ENGN_QUEUE_SIZE)
    queue.last          = 0;
  int idx               = queue.last;
  queue.id[idx]         = id;
  memcpy(&queue.datum[idx], datum, sizeof(IMU_datum));
  
  // exiting critical section
  IMU_thrd_mutex_unlock(&thrdLock);
  
  // exit function (pass error message or queue count)
  if (status < 0)
    return status;
  else 
    return queue.count;
}
#endif


/******************************************************************************
* internal function - processes one datum
******************************************************************************/

int IMU_engn_process(
  uint16_t              id, 
  IMU_datum             *datum)
{
  // define local variables
  IMU_core_FOM          *FOM   = NULL;
  IMU_pnts_entry        *pnt   = NULL;
  IMU_pnts_enum         status;
    
  // create FOM pointer
  if (config[id].isStat || config[id].isFOM)
    FOM = datumFOM;

  // save data to sensor structure
  if (config[id].isSensorStruct)
    IMU_copy_datumRaw(id, datum);
  
  // process datum by subsystems
  if (config[id].isRect)
    state[id].rect = IMU_rect_datum(state[id].idRect, datum);
  if (config[id].isPnts) {
    state[id].pnts = IMU_pnts_datum(state[id].idPnts, datum, &pnt);
    status         = (IMU_pnts_enum)state[id].pnts;
  } else {
    status         = IMU_pnts_enum_move;
  }
  state[id].core   = IMU_core_datum(state[id].idCore, datum, FOM);
  if (config[id].isCalb && pnt != NULL)
    state[id].calb = IMU_calb_point(state[id].idCalb, pnt);
  if (config[id].isStat && FOM != NULL)
    state[id].stat = IMU_stat_datum(state[id].idStat, datum, FOM, status);
    
  // save data to sensor structure
  if (config[id].isSensorStruct)
    IMU_copy_results1(id, datum, FOM);

  // update the datum counter
  state[id].datumCount++;

  // exit function (no errrors)
  if (state[id].rect < 0 || state[id].core < 0 || state[id].pnts < 0 ||
      state[id].calb < 0 || state[id].stat < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
  else
    return 0;
}


/******************************************************************************
* internal function - processes one datum
******************************************************************************/

int IMU_copy_datumRaw(
  uint16_t              id, 
  IMU_datum             *datum)
{
  if      (datum->type == IMU_gyro) 
    memcpy(sensor[id].gRaw, datum->val, sizeof(sensor[id].gRaw));
  else if (datum->type == IMU_accl)
    memcpy(sensor[id].aRaw, datum->val, sizeof(sensor[id].aRaw));
  else if (datum->type == IMU_magn)
    memcpy(sensor[id].mRaw, datum->val, sizeof(sensor[id].mRaw));
  else
    return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
  return 0;
}


/******************************************************************************
* internal function - processes one datum
******************************************************************************/

int IMU_copy_data3Raw(
  uint16_t              id, 
  IMU_data3             *data3)
{
  memcpy(sensor[id].gRaw, data3->g, sizeof(sensor[id].gRaw));
  memcpy(sensor[id].aRaw, data3->a, sizeof(sensor[id].aRaw));
  memcpy(sensor[id].mRaw, data3->m, sizeof(sensor[id].mRaw));
  return 0;
}


/******************************************************************************
* internal function - processes one datum
******************************************************************************/

int IMU_copy_results1(
  uint16_t              id, 
  IMU_datum             *datum,
  IMU_core_FOM          *FOM)
{
  // get IMU_pnts state
  IMU_union_state       unionState;
  IMU_pnts_state        *pntsState;
  if (config[id].isPnts && state[id].configPnts->enable) {
    IMU_engn_getState(id, IMU_engn_pnts, &unionState);
    pntsState = unionState.pnts;
  }

  // copy datum time
  sensor[id].time = datum->t;

  // copy gyroscope info
  if        (datum->type == IMU_gyro) {
    if (config[id].isRect && state[id].configRect->enable)
      memcpy(sensor[id].gCor, datum->val, sizeof(sensor[id].gCor));
    else
      memset(sensor[id].gCor, 0, 3*sizeof(float));
    if (config[id].isPnts && state[id].configPnts->enable)
      memcpy(sensor[id].gFlt, pntsState->current->gFltr, 3*sizeof(float));
    else
      memset(sensor[id].gFlt, 0, 3*sizeof(float));
    if (FOM != NULL && FOM->isValid)
      memcpy(&sensor[id].gFOM, &FOM->FOM.gyro, sizeof(IMU_core_FOM_gyro));
    else
      sensor[id].gFOM = (IMU_core_FOM_gyro){0};
      
  // copy accelerometer info
  } else if (datum->type == IMU_accl) {
    if (config[id].isRect && state[id].configRect->enable)
      memcpy(sensor[id].aCor, datum->val, sizeof(sensor[id].aCor));
    else
      memset(sensor[id].aCor, 0, 3*sizeof(float));
    if (config[id].isPnts && state[id].configPnts->enable)
      memcpy(sensor[id].aFlt, pntsState->current->aFltr, 3*sizeof(float));
    else
      memset(sensor[id].aFlt, 0, 3*sizeof(float));
    if (FOM != NULL && FOM->isValid)
      memcpy(&sensor[id].aFOM, &FOM->FOM.accl, sizeof(IMU_core_FOM_accl));
    else
      sensor[id].aFOM = (IMU_core_FOM_accl){0};
      
  // copy magnetometer info
  } else if (datum->type == IMU_magn) {
    if (config[id].isRect && state[id].configRect->enable)
      memcpy(sensor[id].mCor, datum->val, sizeof(sensor[id].mCor));
    else
      memset(sensor[id].mCor, 0, 3*sizeof(float));
    if (config[id].isPnts && state[id].configPnts->enable)
      memcpy(sensor[id].mFlt, pntsState->current->mFltr, 3*sizeof(float));
    else
      memset(sensor[id].mFlt, 0, 3*sizeof(float));
    if (FOM != NULL && FOM->isValid)
      memcpy(&sensor[id].mFOM, &FOM->FOM.magn, sizeof(IMU_core_FOM_magn));
    else
      sensor[id].mFOM = (IMU_core_FOM_magn){0};
  }

  // exit function (no errors)
  return 0;
}


/******************************************************************************
* internal function - processes one datum
******************************************************************************/

int IMU_copy_results3(
  uint16_t              id, 
  IMU_data3             *data3,
  IMU_core_FOM          *FOM)
{
  // get IMU_pnts state
  IMU_union_state       unionState;
  IMU_pnts_state        *pntsState;
  if (config[id].isPnts) {
    IMU_engn_getState(id, IMU_engn_pnts, &unionState);
    pntsState = unionState.pnts;
  }

  // copy datum time
  sensor[id].time = data3->t;

  // copy rectified data
  if (config[id].isRect && state[id].configRect->enable) {
    memcpy(sensor[id].gCor, data3->g, sizeof(sensor[id].gCor));
    memcpy(sensor[id].aCor, data3->a, sizeof(sensor[id].aCor));
    memcpy(sensor[id].mCor, data3->m, sizeof(sensor[id].mCor));
  } else {
    memset(sensor[id].gCor, 0, 3*sizeof(float));
    memset(sensor[id].aCor, 0, 3*sizeof(float));
    memset(sensor[id].mCor, 0, 3*sizeof(float));
  }

  // copy filtered data
  if (config[id].isPnts && state[id].configPnts->enable) {
    memcpy(sensor[id].aFlt, pntsState->current->aFltr, 3*sizeof(float));
    memcpy(sensor[id].gFlt, pntsState->current->gFltr, 3*sizeof(float));
    memcpy(sensor[id].mFlt, pntsState->current->mFltr, 3*sizeof(float));
  } else {
    memset(sensor[id].gFlt, 0, 3*sizeof(float));
    memset(sensor[id].aFlt, 0, 3*sizeof(float));
    memset(sensor[id].mFlt, 0, 3*sizeof(float));
  }

  // copy figure of merit
  if (FOM != NULL && FOM[0].isValid)
    memcpy(&sensor[id].gFOM, &FOM[0].FOM.gyro, sizeof(IMU_core_FOM_gyro));
  else
    sensor[id].gFOM = (IMU_core_FOM_gyro){0};
  if (FOM != NULL && FOM[1].isValid)
    memcpy(&sensor[id].aFOM, &FOM[1].FOM.accl, sizeof(IMU_core_FOM_accl));
  else
    sensor[id].aFOM = (IMU_core_FOM_accl){0};
  if (FOM != NULL && FOM[2].isValid)
    memcpy(&sensor[id].gFOM, &FOM[2].FOM.magn, sizeof(IMU_core_FOM_magn));
  else
    sensor[id].mFOM = (IMU_core_FOM_magn){0};
  return 0;
}


/******************************************************************************
* internal function - verify subsystem is enabled
******************************************************************************/

int IMU_engn_typeCheck(
  uint16_t              id, 
  IMU_engn_system       system)
{
  if (system == IMU_engn_rect && !config[id].isRect)
    return IMU_ENGN_UNINITIALIZE_SYS;
  if (system == IMU_engn_pnts && !config[id].isPnts)
    return IMU_ENGN_UNINITIALIZE_SYS;
  if (system == IMU_engn_stat && !config[id].isStat)
    return IMU_ENGN_UNINITIALIZE_SYS;
  if (system == IMU_engn_calb && !config[id].isCalb)
    return IMU_ENGN_UNINITIALIZE_SYS;    
  return 0;
}


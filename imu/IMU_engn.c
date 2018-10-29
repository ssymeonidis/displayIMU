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
  uint16_t              id    [IMU_ENGN_QUEUE_SIZE+1];
  IMU_datum             datum [IMU_ENGN_QUEUE_SIZE+1];
  int                   first;
  int                   last;
  int                   count;
} IMU_engn_queue;
#endif

// internally managed structures
static IMU_core_FOM      datumFOM [3];
static IMU_engn_config   config   [IMU_MAX_INST]; 
static IMU_engn_state    state    [IMU_MAX_INST];
static IMU_engn_sensor   sensor   [IMU_MAX_INST];
static uint16_t          numInst = 0;
#if IMU_ENGN_QUEUE_SIZE
static IMU_engn_queue    engnQueue;
#endif
#if IMU_USE_PTHREAD
static useconds_t        engnSleepTime = 20;
static pthread_mutex_t   engnLock;
static int               engnThreadID;
static pthread_t         engnThread;
static uint8_t           engnExitThread;
static uint8_t           engnIsExit;
#endif

// internally defined functions
int IMU_engn_calbFnc   (uint16_t id, IMU_calb_FOM*);
int IMU_engn_process   (uint16_t id, IMU_datum*);
int IMU_copy_datumRaw  (uint16_t id, IMU_datum*);
int IMU_copy_data3Raw  (uint16_t id, IMU_data3*);
int IMU_copy_results1  (uint16_t id, IMU_datum*, IMU_core_FOM*);
int IMU_copy_results3  (uint16_t id, IMU_data3*, IMU_core_FOM*);
int IMU_engn_typeCheck (uint16_t id, IMU_engn_system);
#if IMU_ENGN_QUEUE_SIZE
int IMU_engn_addQueue  (uint16_t id, IMU_datum*);
void* IMU_engn_run     (void*);
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
  config[*id].isSensorStruct = 0;
  config[*id].q_ref[0]       = 1;
  config[*id].q_ref[1]       = 0;
  config[*id].q_ref[2]       = 0;
  config[*id].q_ref[3]       = 0;
  state[*id].rect            = 0;
  state[*id].pnts            = 0;
  state[*id].stat            = 0;
  state[*id].calb            = 0;
  state[*id].quat            = 0;
  state[*id].tran            = 0;
  
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
  if (&cur->core < 0 || &cur->rect < 0 || &cur->pnts < 0 ||
      &cur->stat < 0 || &cur->calb < 0)
    return IMU_ENGN_FAILED_SYS_INIT;

  // create pthread mutex
  #if IMU_USE_PTHREAD
  int err  = IMU_thrd_mutex_init(&engnLock);
  if (err) return IMU_CORE_FAILED_MUTEX;
  #endif

  // exit (no errors)
  printf("finished init\n");
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
  if (!IMU_engn_typeCheck(id, system))
    return IMU_ENGN_UNINITIALIZE_SYS;    

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
  if (IMU_engn_typeCheck(id, system) != 0)
    return IMU_ENGN_UNINITIALIZE_SYS;    

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
  if (IMU_engn_typeCheck(id, system) != 0)
    return IMU_ENGN_UNINITIALIZE_SYS;    

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
  int                   (*pntr)(uint16_t, IMU_calb_FOM*))
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;
  if (!config[id].isCalb)
    return IMU_ENGN_UNINITIALIZE_SYS;

  // pass sensor structure and exit
  return IMU_calb_setFnc(id, pntr);
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
    return IMU_ENGN_FAILED_RESET;

  // zero queue size pass through
  #if IMU_ENGN_QUEUE_SIZE == 0
  return 0;
  #endif
  
  // initialize datum queue
  engnQueue.first = 0;
  engnQueue.last  = IMU_ENGN_QUEUE_SIZE - 1;
  engnQueue.count = 0;
  
  // currently supports pthread only
  #if IMU_USE_PTHREAD
  engnExitThread = 0;
  return pthread_create(&engnThread, NULL, IMU_engn_run, &engnThreadID);
  #else
  return IMU_ENGN_FAILED_THREAD;
  #endif
}


/******************************************************************************
* function to set calibration callback
******************************************************************************/

int IMU_engn_stop()
{
  // zero queue size pass through
  #if IMU_ENGN_QUEUE_SIZE == 0
  return 0;
  #endif
  
  #if IMU_USE_PTHREAD
  engnExitThread = 1;
  while (!engnIsExit)
    usleep(engnSleepTime);
  pthread_join(engnThreadID, NULL);
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
  if (IMU_engn_typeCheck(id, system) != 0)
    return IMU_ENGN_UNINITIALIZE_SYS;    
    
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
  else if (system == IMU_engn_self) 
    return IMU_file_engnLoad(filename, &config[id]);
  else
    return IMU_ENGN_NONEXISTANT_SYSID;
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
  if (IMU_engn_typeCheck(id, system) != 0)
    return IMU_ENGN_UNINITIALIZE_SYS;    
    
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
  else if (system == IMU_engn_self) 
    return IMU_file_engnSave(filename, &config[id]);
  else
    return IMU_ENGN_NONEXISTANT_SYSID;
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
    return IMU_ENGN_FAILED_RESET;
   
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
  config[id].q_ref[0]   = ref[0];
  config[id].q_ref[1]   = ref[1];
  config[id].q_ref[2]   = ref[2];
  config[id].q_ref[3]   = ref[3];
  
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
  config[id].q_ref[0]   = ref[0];
  config[id].q_ref[1]   = ref[1];
  config[id].q_ref[2]   = ref[2];
  config[id].q_ref[3]   = ref[3];
  
  // exit function
  return 0;
}


/******************************************************************************
* function to set reference quaternion (current orientation)
******************************************************************************/

int IMU_engn_calbStart( 
  uint16_t		id,
  IMU_calb_mode         mode)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_ENGN_BAD_INST;

  // get current orientation and pass to setRef
  int status = max(0, IMU_pnts_start(state[id].idPnts, 0));
  status    += max(0, IMU_calb_start(state[id].idCalb, mode));
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // exit function
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
  IMU_engn_state *pntr = &state[id];
  int status = IMU_calb_save(pntr->idCalb, pntr->configRect, pntr->configCore);
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
  IMU_pnts_entry        *pnt;
  IMU_core_FOM          *FOM;
  int                   status = 0;
    
  // save data to sensor structure
  if (config[id].isSensorStruct)
    status = IMU_copy_data3Raw(id, data3);
  if (status < 0)
    return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
  
  // create FOM pointer
  if (config[id].isStat || config[id].isFOM)
    FOM = datumFOM;
  else
    FOM = NULL;

  // process datum by subsystems
  if (config[id].isRect)
    state[id].rect = IMU_rect_data3(state[id].idRect, data3);
  if (config[id].isPnts)
    state[id].pnts = IMU_pnts_data3(state[id].idPnts, data3, &pnt);
  state[id].core   = IMU_core_data3(state[id].idCore, data3, FOM);
  if (config[id].isStat)
    state[id].stat = IMU_stat_update(state[id].idStat, FOM, 3);
  if (state[id].rect < 0 || state[id].pnts < 0 ||
      state[id].core < 0 || state[id].stat < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // save data to sensor structure
  if (config[id].isSensorStruct)
    status = IMU_copy_results3(id, data3, FOM);
  if (status < 0)
    return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
    
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
  
  state[id].quat   = IMU_core_estmQuat(state[id].idCore, t, estm->q_org);
  if (config[id].isEstmAccl)
    state[id].tran = IMU_core_estmAccl(state[id].idCore, t, estm->pos);
  else
    state[id].tran = 0;
  if (!config[id].isQuatOnly) {
    IMU_math_quatMultConj(estm->q_org, config[id].q_ref, estm->q);
    IMU_math_quatToEuler(estm->q, estm->ang);
  }
  
  // exit function
  if (state[id].quat < 0 || state[id].tran < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
  return 0;
}


/******************************************************************************
* "continous run" function handle for recreating a thread 
******************************************************************************/

#if IMU_ENGN_QUEUE_SIZE
void *IMU_engn_run(
  void                  *NA)
{
  // define local variables
  uint16_t              id;
  IMU_datum             datum;

  // main processing loop
  engnIsExit            = 0;
  while(!engnExitThread) {
  
    // wait until queue contains datum
    while (engnQueue.count == 0)
      usleep(engnSleepTime);
    
    // remove datum from queue
    IMU_thrd_mutex_lock(&engnLock);
    id                  = engnQueue.id[engnQueue.first];
    memcpy(&datum, &engnQueue.datum[engnQueue.first], sizeof(IMU_datum));
    engnQueue.count     = engnQueue.count - 1;
    engnQueue.first     = engnQueue.first + 1;
    if (engnQueue.first >= IMU_ENGN_QUEUE_SIZE)
      engnQueue.first   = 0;
    IMU_thrd_mutex_unlock(&engnLock);
          
    // process datum
    IMU_engn_process(id, &datum);
  }
  engnIsExit            = 1;
  return 0;
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
  IMU_thrd_mutex_lock(&engnLock);

  // check queue overflow
  if (engnQueue.count >= IMU_ENGN_QUEUE_SIZE) {
    engnQueue.count     = engnQueue.count - 1;
    engnQueue.first     = engnQueue.first + 1;
    if (engnQueue.first >= IMU_ENGN_QUEUE_SIZE)
      engnQueue.first   = 0; 
    status = IMU_ENGN_QUEUE_OVERFLOW;
  }
  
  // add datum to queue
  engnQueue.count       = engnQueue.count + 1;
  engnQueue.last        = engnQueue.last  + 1;
  if (engnQueue.last >= IMU_ENGN_QUEUE_SIZE)
    engnQueue.last      = 0;
  int idx               = engnQueue.last;
  engnQueue.id[idx]     = id;
  memcpy(&engnQueue.datum[idx], datum, sizeof(IMU_datum));
  
  // exiting critical section
  IMU_thrd_mutex_unlock(&engnLock);
  
  // exit function (pass error message or queue count)
  if (status < 0)
    return status;
  else 
    return engnQueue.count;
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
  IMU_pnts_entry        *pnt;
  IMU_core_FOM          *FOM;
  int                   status = 0;
    
  // save data to sensor structure
  if (config[id].isSensorStruct)
    status = IMU_copy_datumRaw(id, datum);
  if (status < 0)
    return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
  
  // create FOM pointer
  if (config[id].isStat || config[id].isFOM)
    FOM = datumFOM;
  else
    FOM = NULL;

  // process datum by subsystems
  if (config[id].isRect)
    state[id].rect = IMU_rect_datum(state[id].idRect, datum, datum->val);
  if (config[id].isPnts)
    state[id].pnts = IMU_pnts_datum(state[id].idPnts, datum, &pnt);
  state[id].core   = IMU_core_datum(state[id].idCore, datum, FOM);
  if (config[id].isStat)
    state[id].stat = IMU_stat_update(state[id].idStat, FOM, 1);
  if (state[id].rect < 0 || state[id].pnts < 0 ||
      state[id].core < 0 || state[id].stat < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // save data to sensor structure
  if (config[id].isSensorStruct)
    status = IMU_copy_results1(id, datum, FOM);
  if (status < 0)
    return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
    
  // exit function
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
  if (config[id].isPnts) {
    int status = IMU_engn_getState(id, IMU_engn_pnts, &unionState);
    if (status < 0)
      return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
    pntsState = unionState.pnts;
    if (pntsState == NULL)
      return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
  }

  // copy datum time
  sensor[id].time = datum->t;

  // copy gyroscope info
  if        (datum->type == IMU_gyro) {
    if (config[id].isRect && state[id].configRect->enable)
      memcpy(sensor[id].gCor, datum->val, sizeof(sensor[id].gCor));
    else
      memset(sensor[id].gCor, 0, 3*sizeof(float));
    if (config[id].isPnts && pntsState->state == IMU_pnts_enum_stable)
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
    if (config[id].isPnts && pntsState->state == IMU_pnts_enum_stable)
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
    if (config[id].isPnts && pntsState->state == IMU_pnts_enum_stable)
      memcpy(sensor[id].mFlt, pntsState->current->mFltr, 3*sizeof(float));
    else
      memset(sensor[id].mFlt, 0, 3*sizeof(float));
    if (FOM != NULL && FOM->isValid)
      memcpy(&sensor[id].gFOM, &FOM->FOM.magn, sizeof(IMU_core_FOM_magn));
    else
      sensor[id].mFOM = (IMU_core_FOM_magn){0};
  } else {
    return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
  }
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
    int status = IMU_engn_getState(id, IMU_engn_pnts, &unionState);
    if (status < 0)
      return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
    pntsState = unionState.pnts;
    if (pntsState == NULL)
      return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
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
  if (config[id].isPnts && pntsState->state == IMU_pnts_enum_stable) {
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


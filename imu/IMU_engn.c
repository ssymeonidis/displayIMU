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

// definitions for increased readability
#define NULL 0
#define max(a,b)              \
  ({__typeof__ (a) _a = (a);  \
    __typeof__ (b) _b = (b);  \
    _a > _b ? _a : _b; })

// include statements 
#include "IMU_engn.h"

// internally managed structures
IMU_core_FOM      datumFOM [3];
IMU_engn_config   config   [IMU_MAX_INST]; 
IMU_engn_state    state    [IMU_MAX_INST];
IMU_engn_sensor   sensor   [IMU_MAX_INST];
uint16_t          numInstEngn = 0;
#if IMU_ENGN_QUEUE_SIZE
IMU_engn_queue    engnQueue;
#endif
#if IMU_USE_PTHREAD
useconds_t        engnSleepTime = 100;
pthread_mutex_t   engnLock;
int               engnThreadID;
pthread           engnThread;
uint8_t           engnExitThread;
uint8_t           engnIsExit;
#endif

// internally defined functions
int IMU_engn_process  (uint16_t id, IMU_datum*);
int IMU_copy_datumRaw (uint16_t id, IMU_datum*);
int IMU_copy_data3Raw (uint16_t id, IMU_datum*);
int IMU_copy_results1 (uint16_t id, IMU_datum*, IMU_core_FOM*);
int IMU_copy_results3 (uint16_t id, IMU_datum*, IMU_core_FOM*);
#if IMU_ENGN_QUEUE_SIZE
int IMU_engn_addQueue (uint16_t id, IMU_datum*);
void* IMU_engn_run    (void*);
#endif


/******************************************************************************
* function for creating new instance
******************************************************************************/

int IMU_engn_init(
  IMU_engn_type         type,
  uint16_t              *id, 
  IMU_engn_config       **pntr)
{
  // check device count overflow
  if (numInstEngn >= IMU_MAX_INST)
    return IMU_CORE_ENGN_OVERFLOW;

  // initialize config structure
  if        (type == IMU_engn_core_only) {
    config[numInstCore].isRect = 0;
    config[numInstCore].isPnts = 0;
    config[numInstCore].isAuto = 0;
    config[numInstCore].isCalb = 0;
  } else if (type == IMU_engn_rect_core) {
    config[numInstCore].isRect = 1;
    config[numInstCore].isPnts = 0;
    config[numInstCore].isAuto = 0;
    config[numInstCore].isCalb = 0;
  } else if (type == IMU_engn_calb_pnts) {
    config[numInstCore].isRect = 1;
    config[numInstCore].isPnts = 1;
    config[numInstCore].isAuto = 0;
    config[numInstCore].isCalb = 1;
  } else if (type == IMU_engn_calb_auto) {
    config[numInstCore].isRect = 1;
    config[numInstCore].isPnts = 0;
    config[numInstCore].isAuto = 1;
    config[numInstCore].isCalb = 1;
  } else if (type == IMU_engn_calb_full) {
    config[numInstCore].isRect = 1;
    config[numInstCore].isPnts = 1;
    config[numInstCore].isAuto = 1;
    config[numInstCore].isCalb = 1;
  } else {
    return IMU_ENGN_BAD_ENGN_TYPE;
  }
  config[numInstCore].isFOM          = 0;
  config[numInstCore].isSensorStruct = 0;
  
  // create IMU subsystem instances
  int status = IMU_core_init (&state.idCore, &state.configCore);
  if (config[numInstCore].isRect)
    status  += IMU_rect_init (&state.idRect, &state.configRect);
  if (config[numInstCore].isPnts)
    status  += IMU_pnts_init (&state.idPnts, &state.configPnts);
  if (config[numInstCore].isAuto)
    status  += IMU_auto_init (&state.idAuto, &state.configAuto);
  if (config[numInstCore].isCalb)
    status  += IMU_auto_calb (&state.idCalb, &state.configCalb);
  if (status < 0)
    return IMU_ENGN_FAILED_SYS_INIT;
    
  // ensure instance reseted to known state
  status     = IMU_engn_reset(numInstCore);
  if (status < 0)
    return IMU_ENGN_FAILED_RESET;

  // pass handle and config structure
  *id        = numInstCore;
  *pntr      = &config[*id];
  numInstCore++;
  return 0;
}


/******************************************************************************
* function to return config structure
******************************************************************************/

int IMU_engn_getConfig( 
  uint16_t              id,  
  IMU_engn_config       **pntr)
{
  // check out-of-bounds condition
  if (id >= numInstEngn)
    return IMU_ENGN_BAD_INST; 

  // pass config structure and exit
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* function to return state structure
******************************************************************************/

int IMU_engn_getState( 
  uint16_t		id,  
  IMU_core_state        **pntr)
{
  // check out-of-bounds condition
  if (id >= numEngnCore)
    return IMU_ENGN_BAD_INST; 

  // pass state structure and exit
  *pntr = &state[id];
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
  if (id >= numEngnCore)
    return IMU_ENGN_BAD_INST; 

  // pass subsystem handle (id)
  if        (system == IMU_engn_core) {
    *sysID              = state[id].idCore;
  } else if (system == IMU_engn_rect) {
    if (config[id].isRect)
      *sysID            = state[id].idRect;
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
  } else if (system == IMU_engn_pnts) {
    if (config[id].isPnts)
      *sysID            = state[id].idPnts;
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
  } else if (system == IMU_engn_auto) {
    if (config[id].isAuto)
      *sysID            = state[id].idAuto;
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
  } else {
    return IMU_ENGN_NONEXISTANT_SYSID;
  }
  
  // exit (no errors)
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
  if (id >= numEngnCore)
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
  int                   *pntr())
{
  // check out-of-bounds condition
  if (id >= numEngnCore)
    return IMU_ENGN_BAD_INST;
  if (!config[id].isCalb)
    return IMU_ENGN_UNINITIALIZE_SYS;

  // pass sensor structure and exit
  state[id].fncCalb = pntr;
  return 0;
}


/******************************************************************************
* function to set calibration callback
******************************************************************************/

#if IMU_ENGN_QUEUE_SIZE
int IMU_engn_start()
{
  #if IMU_USE_PTHREAD
  engnExitThread = 0;
  return pthread_create(&engnThreadID, NULL, IMU_engn_run, &engnThreadID);
  #else
  return IMU_ENGN_FAILED_THREAD;
  #endif
}
#endif


/******************************************************************************
* function to set calibration callback
******************************************************************************/

#if IMU_ENGN_QUEUE_SIZE
int IMU_engn_stop()
{
  #if IMU_USE_PTHREAD
  engnExitThread = 1;
  while (!engnIsExit)
    usleep(engnSleepTime);
  pthread_join(engnThreadID, NULL);
  #else
  return IMU_ENGN_FAILED_THREAD;
  #endif
}
#endif


/******************************************************************************
* function to load json configuration file
******************************************************************************/

int IMU_engn_load(
  uint16_t              id,
  const char*           filename,
  IMU_engn_system       system)
{
  // check out-of-bounds condition
  if (id >= numEngnCore)
    return IMU_ENGN_BAD_INST;
    
  // load respective json file
  if        (system == IMU_engn_core) {
    return   IMU_file_coreLoad(filename, state[id].configCore);
  } else if (system == IMU_engn_rect) {
    if   (config[id].isRect) 
      return IMU_file_rectLoad(filename, state[id].configRect);
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
  } else if (system == IMU_engn_pnts) {
    if   (config[id].isPnts) 
      return IMU_file_pntsLoad(filename, state[id].configPnts);
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
  } else if (system == IMU_engn_auto) {
    if   (config[id].isAuto) 
      return IMU_file_autoLoad(filename, state[id].configAuto);
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
  } else if (system == IMU_engn_calb) {
    if   (config[id].isCalb) 
      return IMU_ENGN_NONEXISTANT_CONFIG;
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
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
  if (id >= numEngnCore)
    return IMU_ENGN_BAD_INST;
    
  // load respective json file
  if        (system == IMU_engn_core) {
    return   IMU_file_coreSave(filename, state[id].configCore);
  } else if (system == IMU_engn_rect) {
    if   (config[id].isRect) 
      return IMU_file_rectSave(filename, state[id].configRect);
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
  } else if (system == IMU_engn_pnts) {
    if   (config[id].isPnts) 
      return IMU_file_pntsSave(filename, state[id].configPnts);
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
  } else if (system == IMU_engn_auto) {
    if   (config[id].isAuto) 
      return IMU_file_autoSave(filename, state[id].configAuto);
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
  } else if (system == IMU_engn_calb) {
    if   (config[id].isCalb) 
      return IMU_ENGN_NONEXISTANT_CONFIG;
    else
      return IMU_ENGN_UNINITIALIZE_SYS;
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
  if (id >= numInstEngn)
    return IMU_ENGN_BAD_INST; 
    
  // initialize sensor structure
  IMU_TYPE zero_sensor[3] = {0, 0, 0};
  sensor.time           = 0;
  memcpy(sensor[id].gRaw, zero_sensor, sizeof(zero_sensor));
  memcpy(sensor[id].gCor, zero_sensor, sizeof(zero_sensor));
  memcpy(sensor[id].gFlt, zero_sensor, sizeof(zero_sensor));
  memcpy(sensor[id].aRaw, zero_sensor, sizeof(zero_sensor));
  memcpy(sensor[id].aCor, zero_sensor, sizeof(zero_sensor));
  memcpy(sensor[id].aFlt, zero_sensor, sizeof(zero_sensor));
  memcpy(sensor[id].mRaw, zero_sensor, sizeof(zero_sensor));
  memcpy(sensor[id].mCor, zero_sensor, sizeof(zero_sensor));
  memcpy(sensor[id].mFlt, zero_sensor, sizeof(zero_sensor));
  
  // initialize estimate structure
  float zero_float[4] = {0, 0, 0};
  memcpy(estm[id].q_org, zero_float, 4*sizeof(float));
  memcpy(estm[id].q,     zero_float, 4*sizeof(float));
  memcpy(estm[id].ang,   zero_float, 3*sizeof(float));
  memcpy(estm[id].move,  zero_float, 3*sizeof(float));

  // reset all open subsystems
  int status = max(0,IMU_core_reset(state[id].idCore));
  if (config[id].isPnts)
    status  += max(0,IMU_pnts_reset(state[id].idPnts));
  if (config[id].isAuto)
    status  += max(0,IMU_auto_reset(state[id].idAuto));
  if (config[id].isCalb)
    status  += max(0,IMU_calb_reset(state[id].idCalb));
  if (status < 0)
    return IMU_ENGN_RESET_FAIL;
  state[id].fncCalb = NULL;
    
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
  if (id >= numEngnCore)
    return IMU_ENGN_BAD_INST;

  // copy contents of input vector
  state[id].q_ref[0]    = ref[0];
  state[id].q_ref[1]    = ref[1];
  state[id].q_ref[2]    = ref[2];
  state[id].q_ref[3]    = ref[3];
  
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
  if (id >= numEngnCore)
    return IMU_ENGN_BAD_INST;

  // define local variables
  float                 q_ref[4];
  int                   status;

  // get current orientation and pass to setRef
  status = IMU_core_estmQuat(state[id].idCore, q_ref);
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;

  // copy contents of input vector
  state.q_ref[0]        =  ref[0];
  state.q_ref[1]        = -ref[1];
  state.q_ref[2]        = -ref[2];
  state.q_ref[3]        = -ref[3];
  
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
  if (id >= numEngnCore)
    return IMU_ENGN_BAD_INST;

  // get current orientation and pass to setRef
  int status = IMU_calb_start(state[id].idCalb, IMU_calb_mode);
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
  if (id >= numEngnCore)
    return IMU_ENGN_BAD_INST;

  // get current orientation and pass to setRef
  IMU_engn_state *pntr = state[id];
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
  if (id >= numEngnCore)
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
    status = IMU_copy_data3Raw(id, datum);
  if (status < 0)
    return IMU_ENGN_SENSOR_STRUCT_COPY_FAIL;
  
  // create FOM pointer
  if (config[id].isAuto || config[id].isFOM)
    FOM = datumFOM;
  else
    FOM = NULL;

  // process datum by subsystems
  if (config[id].isRect)
    status += max(0,IMU_rect_data3(state[id].idRect, &data3));
  if (config[id].isPnts)
    status += max(0,IMU_pnts_data3(state[id].idPnts, &data3, &pnt));
  status   += max(0,IMU_core_data3(state[id].idCore, &data3, FOM));
  if (config[id].isAuto)
    status += max(0,IMU_auto_process(state[id].idAuto, FOM, 3));
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // save data to sensor structure
  if (config[id].isSensorStruct)
    status = IMU_copy_results3(id, datum);
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
  IMU_engn_estm         *estm)
{
  // get estimates
  int status = max(0,IMU_core_estmQuat(state[id].idCore, estm.q_org));
  if (state[id].isEstmAccl)
    status  += max(0,IMU_core_estmAccl(state[id].idCore, estm.move));
  IMU_math_applyRef(estm.q_org, state[id].q_ref, estm.q);
  IMU_math_quatToEuler(estm.q_org, estm.ang);
  
  // exit function
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
  return 0;
}


/******************************************************************************
* "continous run" function handle for recreating a thread 
******************************************************************************/

void* dataIF_run(
  void                  *NA)
{
  // define local variables
  uint16_t              id;
  IMU_datum             datum;

  // main processing loop
  engnIsExit            = 0;
  while(engnExitThread) {
  
    // wait until queue contains datum
    while (queue.count == 0)
      usleep(engnSleepTime);
    
    // remove datum from queue
    IMU_thrd_mutex_lock(engnLock);
    id                  = queue.id[queue.start];
    memcpy(datum, &queue.datum[queue.first], sizeof(IMU_datum));
    queue.count         = queue.count - 1;
    queue.first         = queue.first + 1;
    IMU_thrd_mutex_unlock(engnLock);
          
    // process datum
    IMU_engn_process(id, datum);
  }
  engnIsExit            = 1;
  return 0;
}


/******************************************************************************
* function to receive a datum
******************************************************************************/

int IMU_engn_addQueue( 
  uint16_t		id,
  IMU_datum             *datum)
{
  // entering critical section
  IMU_thrd_mutex_lock(engnLock);

  // check queue overflow
  if (queue.count >= IMU_ENGN_QUEUE_SIZE) {
    queue.count         = queue.count - 1;
    queue.start         = queue.start + 1;
    if (queue.start >= IMU_ENGN_QUEUE_SIZE)
      queue.start       = 0; 
    status = IMU_ENGN_QUEUE_OVERFLOW;
  }
  
  // add datum to queue
  queue.count           = queue.count + 1;
  queue.last            = queue.last  + 1;
  if (queue.last >= IMU_ENGN_QUEUE_SIZE)
    queue.last          = 0;
  queue.id[queue.last]  = id;
  memcpy(&queue.datum[queue.last], datum, sizeof(IMU_datum));
  
  // exiting critical section
  IMU_thrd_mutex_unlock(engnLock);
}


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
  if (config[id].isAuto || config[id].isFOM)
    FOM = datumFOM;
  else
    FOM = NULL;

  // process datum by subsystems
  if (config[id].isRect)
    status += max(0,IMU_rect_datum(state[id].idRect, &datum, datum->val));
  if (config[id].isPnts)
    status += max(0,IMU_pnts_datum(state[id].idPnts, &datum, &pnt));
  status   += max(0,IMU_core_datum(state[id].idCore, &datum, FOM));
  if (config[id].isAuto)
    status += max(0,IMU_auto_process(state[id].idAuto, FOM, 1));
  if (status < 0)
    return IMU_ENGN_SUBSYSTEM_FAILURE;
    
  // save data to sensor structure
  if (config[id].isSensorStruct)
    status = IMU_copy_results1(id, datum, pnt, FOM);
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
    memcpy(sensor[id].gRaw, datum->val, sizeof(data.gRaw));
  else if (datum->type == IMU_accl)
    memcpy(sensor[id].aRaw, datum->val, sizeof(data.aRaw));
  else if (datum->type == IMU_magn)
    memcpy(sensor[id].mRaw, datum->val, sizeof(data.mRaw));
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
  memcpy(sensor[id].gRaw, datum->g, sizeof(data.gRaw));
  memcpy(sensor[id].aRaw, datum->a, sizeof(data.aRaw));
  memcpy(sensor[id].mRaw, datum->m, sizeof(data.mRaw));
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
  memcpy(sensor[id].gCor, datum->g, sizeof(data.gRaw));
  memcpy(sensor[id].aCor, datum->a, sizeof(data.aRaw));
  memcpy(sensor[id].mCor, datum->m, sizeof(data.mRaw));
  if (FOM[0].isValid)
    memcpy(sensor[id].gFOM, FOM[0].FOM.gyro, sizeof(IMU_core_FOM_gyro));
  else
    sensor[id].gFOM = (IMU_core_FOM_gyro){0};
  if (FOM[1].isValid)
    memcpy(sensor[id].aFOM, FOM[1].FOM.accl, sizeof(IMU_core_FOM_accl));
  else
    sensor[id].aFOM = (IMU_core_FOM_accl){0};
  if (FOM[2].isValid)
    memcpy(sensor[id].gFOM, FOM[2].FOM.magn, sizeof(IMU_core_FOM_magn));
  else
    sensor[id].mFOM = (IMU_core_FOM_magn){0};
  return 0;
}

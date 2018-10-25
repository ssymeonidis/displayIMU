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

/*
 * This function is based off the work performed by Sebastian O.H. Madgwick, 
 * documented in the paper "An efficient orientation Filter for inertial and
 * inertial/magnetic sensor arrays". Changes were made to support async data
 * and adapted the filter for activity recogition and wearable sensors.
*/  


// definitions (increased readability)
#define NULL 0

// include statements 
#include <math.h> 
#include <stdlib.h>
#include <string.h>
#include "IMU_thrd.h"
#include "IMU_math.h"
#include "IMU_core.h"

// internally managed structures
static IMU_core_config  config [IMU_MAX_INST]; 
static IMU_core_state   state  [IMU_MAX_INST];
static IMU_core_FOM     staticFOM;
static uint16_t         numInst = 0;

// internal functions definitions
static inline float  norm3(IMU_TYPE *in, float *out);
static inline float* scale(float *v, float m);  
int IMU_core_newGyro (uint16_t id, float t, IMU_TYPE *g, IMU_core_FOM*);
int IMU_core_newAccl (uint16_t id, float t, IMU_TYPE *a, IMU_core_FOM*);
int IMU_core_newMagn (uint16_t id, float t, IMU_TYPE *m, IMU_core_FOM*);
int IMU_core_zero    (uint16_t id, float t, IMU_TYPE *a, IMU_TYPE *m);


/******************************************************************************
* initialize new instance (constructor) 
******************************************************************************/

int IMU_core_init(
  uint16_t              *id, 
  IMU_core_config       **pntr)
{
  // check device count overflow
  if (numInst >= IMU_MAX_INST)
    return IMU_CORE_INST_OVERFLOW;

  // create pthread mutex
  #if IMU_USE_PTHREAD
  int err  = IMU_thrd_mutex_init(&state[numInst].lock);
  if (err) return IMU_CORE_FAILED_MUTEX;
  #endif

  // pass handle and config pointer
  *id      = numInst; 
  *pntr    = &config[*id];
  numInst++;

  // exit function (no errors)
  return 0;
}


/******************************************************************************
* return config structure
******************************************************************************/

int IMU_core_getConfig( 
  uint16_t              id,  
  IMU_core_config        **pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST; 

  // pass config and exit (no errors)
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* return state structure
******************************************************************************/

int IMU_core_getState( 
  uint16_t		id,  
  IMU_core_state        **pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST; 

  // pass state and exit (no errors)
  *pntr = &state[id];
  return 0;
}


/******************************************************************************
* initialize state and autocal to known state
******************************************************************************/

int IMU_core_reset(
  uint16_t              id)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST; 

  // lock before modifying state
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_lock(&state[id].lock);
  #endif

  // initialize state to known value
  state[id].q[0]        = 1.0;
  state[id].q[1]        = 0.0;
  state[id].q[2]        = 0.0;
  state[id].q[3]        = 0.0;
  state[id].aReset      = config[id].isAccl;
  state[id].mReset      = config[id].isMagn;
  state[id].estmValid   = 0;

  // unlock function and exit (no errors)
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_unlock(&state[id].lock);
  #endif
  return 0;
}


/******************************************************************************
* core function for dead recon
******************************************************************************/

int IMU_core_zero(
  uint16_t              id,  
  float                 t,
  IMU_TYPE              *a_in, 
  IMU_TYPE              *m_in)
{
  // lock before modifying state
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_lock(&state[id].lock);
  #endif

  // copy vectors to have the correct type
  float                 a[3], m[3];
  if (a_in != NULL) {   
    a[0]                = (float)a_in[0]; 
    a[1]                = (float)a_in[1];
    a[2]                = (float)a_in[2];
  }
  if (m_in != NULL) {   
    m[0]                = (float)m_in[0]; 
    m[1]                = (float)m_in[1];
    m[2]                = (float)m_in[2];
  }

  // zero with accerometer and magnetometer
  if        ( config[id].isAccl && config[id].isMagn) {

    // synced sensor data (datum3)
    if        (a!=NULL && m!=NULL) {
      IMU_math_upFrwdToQuat(a, m, state[id].q);
      state[id].aReset  = 0;
      state[id].mReset  = 0;

    // asynchnous accelerometer vector
    } else if (a!=NULL && m==NULL) {
      if (state[id].mReset)
        IMU_math_upToQuat(a, state[id].q);
      else
        IMU_math_upFrwdToQuat(a, state[id].mInit, state[id].q);
      state[id].aReset  = 0;
      
    // asynchnous magnetometer vector
    } else if (a==NULL && m!=NULL) {
      if (state[id].aReset) {
        memcpy(state[id].mInit, m, 3*sizeof(IMU_TYPE));
      } else {
        IMU_math_quatToUp(state[id].q, a);
        IMU_math_upFrwdToQuat(a, m, state[id].q);
      }
      state[id].mReset  = 0;
    
    // no sensor data provided
    } else {
      return IMU_CORE_FNC_DISABLED;
    }

  // no magnetometer configuation
  } else if ( config[id].isAccl && !config[id].isMagn) {
    if (a==NULL || !state[id].aReset)
      return IMU_CORE_FNC_DISABLED;
    IMU_math_upToQuat(a, state[id].q);
    state[id].aReset    = 0;

  // no accelerometer configuration
  } else if (!config[id].isAccl &&  config[id].isMagn) {
    if (m==NULL || !state[id].mReset)
      return IMU_CORE_FNC_DISABLED;
    a[0]                = 0.0f;
    a[1]                = 0.0f;
    a[2]                = 1.0f;
    IMU_math_upFrwdToQuat(a, m, state[id].q);
    state[id].mReset    = 0;

  // gyroscope only configuration
  } else {
    state[id].q[0]      = 1.0f;
    state[id].q[1]      = 0.0f;
    state[id].q[2]      = 0.0f;
    state[id].q[3]      = 0.0f;
   }

  // unlock function and exit (no errors)
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_unlock(&state[id].lock);
  #endif
  return 0;
}


/******************************************************************************
* initialize state and autocal to known state
******************************************************************************/

int IMU_core_datum(
  uint16_t              id,
  IMU_datum             *datum,
  IMU_core_FOM          *FOM)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST; 
    
  // set FOM pointer to datum
  if (FOM != NULL)
    FOM->pntr.datum     = datum;

  // check sensor type and execute
  if      (datum->type == IMU_gyro)
    return IMU_core_newGyro(id, datum->t, datum->val, FOM);
  else if (datum->type == IMU_accl)
    return IMU_core_newAccl(id, datum->t, datum->val, FOM);
  else if (datum->type == IMU_magn)
    return IMU_core_newMagn(id, datum->t, datum->val, FOM);
    
  // exit fucntion (no errors)
  return 0;
}


/******************************************************************************
* estimate euler angle given gyroscope, accelerometer, and magnetometer data 
******************************************************************************/

int IMU_core_data3(
  uint16_t              id, 
  IMU_data3             *data3,
  IMU_core_FOM          *FOM)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST; 
    
  // set FOM pointer to data3
  if (FOM != NULL)
    FOM->pntr.data3     = data3;

  // check reset conditions
  if (state[id].mReset || state[id].aReset) {

    // initialize to known value
    if (FOM != NULL) {
      FOM[0].isValid    = 0;
      FOM[1].isValid    = 0;
      FOM[2].isValid    = 0;
    }
    
    // zero the system to the current sensor
    IMU_core_zero(id, data3->t, data3->a, data3->m);
    return IMU_CORE_FNC_ZEROED;
  }
  
  // update system state w/ each sensor
  if (FOM != NULL) {
    IMU_core_newGyro(id, data3->t, data3->g, &FOM[0]);
    IMU_core_newAccl(id, data3->t, data3->a, &FOM[1]);
    IMU_core_newMagn(id, data3->t, data3->m, &FOM[2]);
  } else {
    IMU_core_newGyro(id, data3->t, data3->g, NULL);
    IMU_core_newAccl(id, data3->t, data3->a, NULL);
    IMU_core_newMagn(id, data3->t, data3->m, NULL);
  }
    
  // exit fucntion (no errors)
  return 0;
}


/******************************************************************************
* apply gyroscope rates
******************************************************************************/

int IMU_core_newGyro(
  uint16_t              id, 
  float                 t, 
  IMU_TYPE              *g_in,
  IMU_core_FOM          *pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST; 

  // initialize figure of merit
  IMU_core_FOM_gyro     *FOM;
  if (pntr != NULL) {
    pntr->isValid       = 1;
    FOM                 = &pntr->FOM.gyro;
  } else {
    FOM                 = &staticFOM.FOM.gyro;
  }
  
  // determine whether the function needs to be executed
  if (!config[id].enable || !config[id].isGyro)
    return IMU_CORE_FNC_DISABLED;
  
  // copy values and calcuate mag 
  float g[3]            = {(float)g_in[0], (float)g_in[1], (float)g_in[2]};
  FOM->magSqrd          = g[0]*g[0] + g[1]*g[1] + g[2]*g[2];
 
  // check stable condition
  FOM->isStable         = 0;
  if (config[id].isStable) {
    if (FOM->magSqrd > config[id].gThresh * config[id].gThresh)
      state[id].tMove   = t;
    else if (t - state[id].tMove > config[id].gThreshTime) {
      FOM->isStable     = 1;
      return IMU_CORE_GYRO_STABLE;
    }
  }

  // lock before modifying state
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_lock(&state[id].lock);
  #endif

  // compute gyro quaternion rate and apply delta to estimated orientation
  float *q       = state[id].q;
  float half_dt  = 0.5 * (t - state[id].t);
  q[0]          += half_dt * (-q[1]*g[0] - q[2]*g[1] - q[3]*g[2]);
  q[1]          += half_dt * ( q[0]*g[0] + q[2]*g[2] - q[3]*g[1]);
  q[2]          += half_dt * ( q[0]*g[1] - q[1]*g[2] + q[3]*g[0]);
  q[3]          += half_dt * ( q[0]*g[2] + q[1]*g[1] - q[2]*g[0]); 
  state[id].t    = t;

  // unlock mutex and exit (no errors)
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_unlock(&state[id].lock);
  #endif
  return 0;
}


/******************************************************************************
* apply accelerometer vector
******************************************************************************/

int IMU_core_newAccl(
  uint16_t              id,
  float                 t,
  IMU_TYPE              *a_in,
  IMU_core_FOM          *pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST;

  // initialize figure of merit
  IMU_core_FOM_accl     *FOM;
  if (pntr != NULL) {
    pntr->isValid       = !state[id].aReset && config[id].isFOM;
    FOM                 = &pntr->FOM.accl;
  } else {
    FOM                 = &staticFOM.FOM.accl;
  }
  
  // determine whether the functions needs to be executed
  if (!config[id].enable || !config[id].isAccl)
    return IMU_CORE_FNC_DISABLED;
  if (state[id].aReset) {
    IMU_core_zero(id, t, a_in, state[id].mInit);
    return IMU_CORE_FNC_ZEROED;
  }

  // normalize input vector
  float a[3];
  FOM->mag              = norm3(a_in, a);
  
  // determine datum quality (based on amplitude)
  if (config[id].isFOM) {
    float ref           = config[id].aMag;
    float thresh        = config[id].aMagThresh;
    FOM->magFOM         = IMU_math_calcWeight(FOM->mag, ref, thresh);
    if (FOM->magFOM <= 0) 
      return IMU_CORE_NO_WEIGHT;
  }

  // copy internal orientation state
  float *q, *A, t_copy;
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_lock(&state[id].lock);
  memcpy(q, state[id].q, sizeof(state[id].q));
  if (config[id].isPos)
    memcpy(A, state[id].A, sizeof(state[id].A));
  t_copy                = state[id].t;
  IMU_thrd_mutex_unlock(&state[id].lock);
  #else
  q                     = state[id].q;
  A                     = state[id].A;
  state[id].t           = t;
  #endif

  // save accelerometer data
  if (config[id].isPos) {
    float   G[3];       
    scale(IMU_math_quatToUp(state[id].q, G), config[id].aMag);
    uint8_t *valid      = &state[id].estmValid;
    if (!state[id].aReset && !state[id].mReset && *valid) {
      A[0]              = (float)a_in[0]-G[0];
      A[1]              = (float)a_in[1]-G[1];
      A[2]              = (float)a_in[2]-G[2];
      *valid            = 1;
    } else {
      float alpha       = config[id].posAlpha * FOM->magFOM;
      A[0]              = alpha*A[0] + (1.0f-alpha)*((float)a_in[0]-G[0]);
      A[1]              = alpha*A[1] + (1.0f-alpha)*((float)a_in[1]-G[1]);
      A[2]              = alpha*A[2] + (1.0f-alpha)*((float)a_in[2]-G[2]);
    }
  }

  // update system state (quaternion)
  float weight = config[id].aWeight * FOM->magFOM;
  int status   = IMU_math_estmAccl(q, a, weight, &FOM->delt);
  
  // save results to system state
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_lock(&state[id].lock);
  memcpy(state[id].q, q, sizeof(state[id].q));
  if (config[id].isPos)
    memcpy(state[id].A, A, sizeof(state[id].A));
  float t_new           = FOM->magFOM * t_copy + (1.0 - FOM->magFOM) * t;
  if (t_new > state[id].t)
    state[id].t         = t_new;
  IMU_thrd_mutex_unlock(&state[id].lock);
  #endif

  // pass status and exit function
  return status;
}


/******************************************************************************
* apply magnetometer vector
******************************************************************************/

int IMU_core_newMagn(
  uint16_t              id, 
  float                 t,
  IMU_TYPE              *m_in,
  IMU_core_FOM          *pntr)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST; 

  // initialize figure of merit
  IMU_core_FOM_magn     *FOM;
  if (pntr != NULL) {
    pntr->isValid       = !state[id].mReset && config[id].isFOM;
    FOM                 = &pntr->FOM.magn;               
  } else {
    FOM                 = &staticFOM.FOM.magn;
  }
  
  // determine whether the functions needs to be executed
  if (!config[id].isMagn || !config[id].enable)
    return IMU_CORE_FNC_DISABLED;
  if (state[id].mReset) {
    IMU_core_zero(id, t, NULL, m_in);
    return IMU_CORE_FNC_ZEROED;
  }

  // normalize input vector
  float m[3];
  FOM->mag              = norm3(m_in, m);

  // determine datum quality factor
  float weight          = 1.0;
  if (config[id].isFOM) {

    // determine magnitude error
    float ref           = config[id].mMag;
    float thresh        = config[id].mMagThresh;
    FOM->magFOM         = IMU_math_calcWeight(FOM->mag, ref, thresh);
    
    // determine angle error
    ref                 = config[id].mAng;
    thresh              = config[id].mAngThresh;
    float a[3];
    IMU_math_quatToUp(state[id].q, a);
    FOM->ang            = acosf(a[0]*m[0] + a[1]*m[1] + a[2]*m[2]);
    FOM->angFOM         = IMU_math_calcWeight(FOM->ang, ref, thresh);

    // check zero weight conditiond
    weight              = FOM->magFOM * FOM->angFOM;
    if (weight <= 0)
      return IMU_CORE_NO_WEIGHT;
  }
  
  // copy internal orientation state
  float *q, t_copy;
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_lock(&state[id].lock);
  memcpy(q, state[id].q, sizeof(state[id].q));
  t_copy                = state[id].t;
  IMU_thrd_mutex_unlock(&state[id].lock);
  #else
  q                     = state[id].q;
  state[id].t           = t;
  #endif

  // update system state (quaternion)
  float weightTemp = weight*config[id].mWeight;
  int status = IMU_math_estmMagnRef(q, m, 1.0, 0.0, weightTemp, &FOM->delt);
    
  // save results to system state
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_lock(&state[id].lock);
  memcpy(state[id].q, q, sizeof(state[id].q));
  float t_new           = weight * t_copy + (1.0f - weight) * t;
  if (t_new > state[id].t)
    state[id].t         = t_new;
  IMU_thrd_mutex_unlock(&state[id].lock);
  #endif

  // pass status and exit function
  return status;
}


/******************************************************************************
* estimate orientation (returns quaternion)
******************************************************************************/

int IMU_core_estmQuat(
  uint16_t              id,
  float                 t,
  float                 *estm)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST; 
  if (!config[id].enable)
    return IMU_CORE_FNC_DISABLED;

  // check reset condition
  if (state[id].aReset || state[id].mReset) {
    memset(estm, 0, 4*sizeof(float));
    return IMU_CORE_FNC_ZEROED;
  }

  // lock before copying state
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_lock(&state[id].lock);
  #endif

  // copy current orientation state and return
  memcpy(estm, state[id].q, 4*sizeof(float));

  // unlock mutex and exit
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_unlock(&state[id].lock);
  #endif
  return 0;
}


/******************************************************************************
* estimate acceleration (minus gravity)
******************************************************************************/

int IMU_core_estmAccl(
  uint16_t              id,
  float                 t,
  float                 *estm)
{
  // check out-of-bounds condition
  if (id >= numInst)
    return IMU_CORE_BAD_INST; 
  if (!config[id].enable)
    return IMU_CORE_FNC_DISABLED;

  // check reset condition
  if (state[id].aReset || state[id].mReset) {
    memset(estm, 0, 3*sizeof(float));
    return IMU_CORE_FNC_ZEROED;
  }

  // lock before copying state
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_lock(&state[id].lock);
  #endif

  // copy internal variables 
  float A[3];  memcpy(A, state[id].A, sizeof(A));
  float q[4];  memcpy(q, state[id].q, sizeof(q));

  // unlock mutex
  #if IMU_USE_PTHREAD
  IMU_thrd_mutex_unlock(&state[id].lock);
  #endif

  // apply rotation to acceleration vector
  float tmp[4]          = {-q[1]*A[0] - q[2]*A[1] - q[3]*A[2],
                            q[0]*A[0] + q[2]*A[2] - q[3]*A[1],
                            q[0]*A[1] - q[1]*A[2] + q[3]*A[0],
                            q[0]*A[2] + q[1]*A[1] - q[2]*A[0]};
  estm[0] = -tmp[0]*q[1] + tmp[1]*q[0] - tmp[2]*q[3] + tmp[3]*q[2];
  estm[1] = -tmp[0]*q[2] + tmp[1]*q[3] + tmp[2]*q[0] - tmp[3]*q[1];
  estm[2] = -tmp[0]*q[3] - tmp[1]*q[2] + tmp[2]*q[1] + tmp[3]*q[0];
  
  // exit function (no errors)
  return 0;
}


/******************************************************************************
* utility function - normalize 3x1 array
******************************************************************************/

inline float norm3(
  IMU_TYPE       *in, 
  float          *out)
{
  float norm     = sqrtf((float)in[0]*(float)in[0] + 
                         (float)in[1]*(float)in[1] + 
                         (float)in[2]*(float)in[2]); 
  out[0]         = (float)in[0] / norm;
  out[1]         = (float)in[1] / norm;
  out[2]         = (float)in[2] / norm;
  return norm;
}


/******************************************************************************
* utility function - scale 4x1 array by scalar value
******************************************************************************/

inline float* scale(
  float          *v, 
  float          m)
{
  v[0]          *= m;
  v[1]          *= m;
  v[2]          *= m;
  v[3]          *= m;
  return v;     // allows function to be used as function argument
}

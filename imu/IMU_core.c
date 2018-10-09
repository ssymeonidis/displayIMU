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

/*
 * This function is based off the work performed by Sebastian O.H. Madgwick, 
 * documented in the paper "An efficient orientation Filter for inertial and
 * inertial/magnetic sensor arrays. Changes were made to turn on/off sensors 
 * and to adapted the filter for human activity recoginition.  Significant
 * changes were made for adding hooks and increasing readabilty 
*/

// definitions for increased readability
#define NULL 0

// include statements 
#include <math.h> 
#include <stdlib.h>
#include <string.h>
#include "IMU_math.h"
#include "IMU_core.h"

// internally managed structures
IMU_core_config  config [IMU_MAX_INST]; 
IMU_core_state   state  [IMU_MAX_INST];
IMU_core_FOM     staticFOM;
uint16_t         numInstCore = 0;

// internally defined functions
inline float  norm3(IMU_TYPE *in, float *out);
inline float* norm4(float *v);
inline float* scale(float *v, float m);  
inline float* decrm(float *v, float *d);
int IMU_core_newGyro (uint16_t id, float t, IMU_TYPE *g, IMU_core_FOM*);
int IMU_core_newAccl (uint16_t id, float t, IMU_TYPE *a, IMU_core_FOM*);
int IMU_core_newMagn (uint16_t id, float t, IMU_TYPE *m, IMU_core_FOM*);
int IMU_core_zero    (uint16_t id, float t, IMU_TYPE *a, IMU_TYPE *m);


/******************************************************************************
* utility function - used prior to entering critical section 
******************************************************************************/

inline int mutex_init(
  uint16_t       id)
{
  #if IMU_USE_PTHREAD
  pthread_mutex_t *lock = &state[id].lock;
  return pthread_mutex_init(lock, NULL);
  #else
  return 0;
  #endif
}


/******************************************************************************
* utility function - used prior to entering critical section 
******************************************************************************/

inline void mutex_lock(
  uint16_t       id)
{
  #if IMU_USE_PTHREAD
  pthread_mutex_lock(&state[id].lock);
  #endif
}


/******************************************************************************
* utility function - used prior to leaving critical section
******************************************************************************/

inline void mutex_unlock(
  uint16_t       id)
{
  #if IMU_USE_PTHREAD
  pthread_mutex_unlock(&state[id].lock);
  #endif
}


/******************************************************************************
* function to return config structure handle
******************************************************************************/

int IMU_core_init(
  uint16_t              *id, 
  IMU_core_config       **pntr)
{
  // check for device count overflow
  if (numInstCore >= IMU_MAX_INST)
    return IMU_CORE_INST_OVERFLOW;

  // create pthread mutex
  int err  = mutex_init(numInstCore);
  if (err) return IMU_CORE_FAILED_MUTEX;

  // return handle and calib pointer
  *id      = numInstCore; 
  *pntr    = &config[*id];
  numInstCore++;
  return 0;
}


/******************************************************************************
* function to copy config structure
******************************************************************************/

int IMU_core_getConfig( 
  uint16_t              id,  
  IMU_core_config        **pntr)
{
  // check for out-of-bounds condition
  if (id > numInstCore-1)
    return IMU_CORE_BAD_INST; 

  // return state
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* function to copy state structure
******************************************************************************/

int IMU_core_getState( 
  uint16_t		id,  
  IMU_core_state        **pntr)
{
  // check for out-of-bounds condition
  if (id > numInstCore-1)
    return IMU_CORE_BAD_INST; 

  // return state
  *pntr = &state[id];
  return 0;
}


/******************************************************************************
* initialize state and autocal to known state
******************************************************************************/

int IMU_core_reset(
  uint16_t              id)
{
  // check for out-of-bounds condition
  if (id > numInstCore-1)
    return IMU_CORE_BAD_INST; 

  // lock before modifying state
  mutex_lock(id);

  // initialize state to known value
  state[id].SEq[0]      = 1.0;
  state[id].SEq[1]      = 0.0;
  state[id].SEq[2]      = 0.0;
  state[id].SEq[3]      = 0.0;
  state[id].aReset      = config[id].isAccl;
  state[id].mReset      = config[id].isMagn;
  state[id].estmValid   = 0;

  // unlock function and exit
  mutex_unlock(id);
  return 0;
}


/******************************************************************************
* core function for dead recon
* (this function needs verification)
******************************************************************************/

int IMU_core_zero(
  uint16_t              id,  
  float                 t,
  IMU_TYPE              *a_in, 
  IMU_TYPE              *m_in)
{
  // check for out-of-bounds condition
  if (id > numInstCore-1)
    return IMU_CORE_BAD_INST;
  if (!config[id].enable)
    return IMU_CORE_FNC_DISABLED;
    
  // lock before modifying state
  mutex_lock(id);

  // normalize accerometer data or derive from system state
  float                 a[3];
  if (!config[id].isAccl || a_in == NULL) {
    IMU_math_quatToUp(state[id].SEq, a);
  } else {
    norm3(a_in, a);
    state[id].aReset    = 0;
  }
  
  // normalize magnetometer data or derive from system state
  float                 m[3];
  if (!config[id].isMagn || m == NULL) {
    IMU_math_quatToFrwd(state[id].SEq, m);
  } else {
    norm3(m_in, m);
    state[id].mReset    = 0;
  }
  
  // calcuate orientation and update state
  IMU_math_upFrwdToQuat(a, m, state[id].SEq);
  state[id].t           = t;

  // unlock function and exit 
  mutex_unlock(id);
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
  // check sensor type and execute
  if      (datum->type == IMU_gyro)
    return IMU_core_newGyro(id, datum->t, datum->val, FOM);
  else if (datum->type == IMU_accl)
    return IMU_core_newAccl(id, datum->t, datum->val, FOM);
  else if (datum->type == IMU_magn)
    return IMU_core_newMagn(id, datum->t, datum->val, FOM);
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
  // check for out-of-bounds condition
  if (id > numInstCore - 1)
    return IMU_CORE_BAD_INST; 
    
  // check for reset conditions
  if (state[id].mReset || state[id].aReset) {
    // initialize to known value
    if (data3->FOM != NULL) {
      // initialize figure of merits
      data3->FOM[0].isValid    = 0;
      data3->FOM[1].isValid    = 0;
      data3->FOM[2].isValid    = 0;
    }
    
    // zero the system to the current sensor
    IMU_core_zero(id, data3->t, data3->a, data3->m);
    return IMU_CORE_FNC_ZEROED;
  }
  
  // update system state w/ each sensor
  if (data3->FOM != NULL) {
    IMU_core_newGyro(id, data3->t, data3->g, &FOM[0]);
    IMU_core_newAccl(id, data3->t, data3->a, &FOM[1]);
    IMU_core_newMagn(id, data3->t, data3->m, &FOM[2]);
  } else {
    IMU_core_newGyro(id, data3->t, data3->g, NULL);
    IMU_core_newAccl(id, data3->t, data3->a, NULL);
    IMU_core_newMagn(id, data3->t, data3->m, NULL);
  }
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
  // check for out-of-bounds condition
  if (id > numInstCore-1)
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
  if (state[id].aReset   || state[id].mReset)
    return IMU_CORE_FNC_IN_RESET;
  
  // copy values and calcuate mag 
  float g[3]            = {(float)g_in[0], (float)g_in[1], (float)g_in[2]};
  FOM->magSqrd          = g[0]*g[0] + g[1]*g[1] + g[2]*g[2];
 
  // check for stable condition
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
  mutex_lock(id);

  // compute gyro quaternion rate and apply delta to estimated orientation
  float *SEq     = state[id].SEq;
  float half_dt  = 0.5 * (t - state[id].t);
  SEq[0]        += half_dt * (-SEq[1]*g[0] - SEq[2]*g[1] - SEq[3]*g[2]);
  SEq[1]        += half_dt * ( SEq[0]*g[0] + SEq[2]*g[2] - SEq[3]*g[1]);
  SEq[2]        += half_dt * ( SEq[0]*g[1] - SEq[1]*g[2] + SEq[3]*g[0]);
  SEq[3]        += half_dt * ( SEq[0]*g[2] + SEq[1]*g[1] - SEq[2]*g[0]); 
  state[id].t    = t;

  // unlock function and exit
  mutex_unlock(id);
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
  // check for out-of-bounds condition
  if (id > numInstCore - 1)
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
    IMU_core_zero(id, t, a_in, NULL);
    return IMU_CORE_FNC_ZEROED;
  }

  // normalize input vector
  float a[3];
  FOM->mag              = norm3(a_in, a);
  
  // determine datum quality factor
  float aWeight         = config[id].aWeight;
  if (config[id].isFOM) {
    float error         = abs(FOM->mag - config[id].aMag) / config[id].aMag;
    FOM->magFOM         = 1.0 - error / config[id].aMagThresh;
    FOM->magFOM         = (FOM->magFOM < 0.0) ? 0.0 : FOM->magFOM;
    aWeight            *= FOM->magFOM;
  }
    
  // check reset and zero weight condition
  if (aWeight <= 0) 
    return IMU_CORE_NO_WEIGHT;

  // copy internal orientation state
  mutex_lock(id);  
  float SEq[4], A[3];  
  memcpy(SEq, state[id].SEq, sizeof(SEq));
  if (config[id].isMove)
    memcpy(A, state[id].A,   sizeof(A));
  float t_copy          = state[id].t;
  mutex_unlock(id);
    
  // save accelerometer data
  if (config[id].isMove) {
    mutex_lock(id);  
    float   G[3];       
    scale(IMU_math_quatToUp(state[id].SEq, G), config[id].aMag);
    uint8_t *valid      = &state[id].estmValid;
    if (!state[id].aReset && !state[id].mReset && *valid) {
      A[0]              = (float)a_in[0]-G[0];
      A[1]              = (float)a_in[1]-G[1];
      A[2]              = (float)a_in[2]-G[2];
      *valid            = 1;
    } else {
      float alpha       = config[id].moveAlpha * aWeight;
      A[0]              = alpha*A[0] + (1.0f-alpha)*((float)a_in[0]-G[0]);
      A[1]              = alpha*A[1] + (1.0f-alpha)*((float)a_in[1]-G[1]);
      A[2]              = alpha*A[2] + (1.0f-alpha)*((float)a_in[2]-G[2]); 
    }
    mutex_unlock(id);
  }

  // compute the objective function 
  float twoSEq[4]       = {2.0f*SEq[0], 2.0f*SEq[1], 2.0f*SEq[2], 2.0f*SEq[3]};
  float f_1             = twoSEq[1]*SEq[3] - twoSEq[0]*SEq[2] - a[0];
  float f_2             = twoSEq[0]*SEq[1] + twoSEq[2]*SEq[3] - a[1];
  float f_3             = 1.0f - twoSEq[1]*SEq[1] - twoSEq[2]*SEq[2] - a[2];
 
  // compute the Jacobian
  float J_11or24        = twoSEq[2]; 
  float J_12or23        = twoSEq[3];
  float J_13or22        = twoSEq[0];
  float J_14or21        = twoSEq[1];
  float J_32            = 2.0f * J_14or21;
  float J_33            = 2.0f * J_11or24;

  // calculate the gradient
  float SEqHatDot[4]    = {J_14or21 * f_2 - J_11or24 * f_1,
                           J_12or23 * f_1 + J_13or22 * f_2 - J_32     * f_3,
                           J_12or23 * f_2 - J_33     * f_3 - J_13or22 * f_1,
                           J_14or21 * f_1 + J_11or24 * f_2};
  norm4(decrm(SEq, scale(norm4(SEqHatDot), aWeight)));
  FOM->delt             = SEqHatDot[0];
  
  // update state
  mutex_lock(id);  
  memcpy(state[id].SEq, SEq, sizeof(SEq));
  if (config[id].isMove)
    memcpy(state[id].A, A, sizeof(A));
  float t_new           = aWeight * t + (1.0 - aWeight) * t_copy;
  if (t_new > state[id].t)
    state[id].t         = t_new;
  mutex_unlock(id);

  // exit function
  return 0;
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
  // check for out-of-bounds condition
  if (id > numInstCore-1)
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

  // copy internal orientation state
  mutex_lock(id);
  float                 SEq[4];
  memcpy(SEq, state[id].SEq, 4*sizeof(float));
  mutex_unlock(id);

  // determine datum quality factor
  float mWeight         = config[id].mWeight;
  if (config[id].isFOM) {
    // determine magnitude error
    float error         = abs(FOM->mag - config[id].mMag) / config[id].mMag;
    FOM->magFOM         = 1.0 - error / config[id].mMagThresh;
    FOM->magFOM         = (FOM->magFOM < 0.0) ? 0.0 : FOM->magFOM;
    mWeight             *= FOM->magFOM;
    
    // determine angle error
    float a[3];
    IMU_math_quatToUp(state[id].SEq, a);
    FOM->ang            = acos(a[0]*m[0] + a[1]*m[1] + a[2]*m[2]);
    error               = abs(FOM->ang - config[id].mAng) / config[id].mAng;
    FOM->angFOM         = 1.0 - error / config[id].mAngThresh;
    FOM->angFOM         = (FOM->angFOM < 0.0) ? 0.0 : FOM->angFOM;
    mWeight            *= FOM->angFOM;
  }
  
  // check zero weight conditiond
  if (mWeight <= 0)
    return IMU_CORE_NO_WEIGHT;

  // compute the objective function 
  float twom_x          = 2.0f * m[0];
  float twom_z          = 2.0f * m[2];
  float SEq_1SEq_3      = SEq[0] * SEq[2];
  float SEq_2SEq_4      = SEq[1] * SEq[3];
  float f_4             = twom_x * (0.5 - SEq[2]*SEq[2] - SEq[3]*SEq[3]) + 
                          twom_z * (SEq_2SEq_4 - SEq_1SEq_3) - m[0];
  float f_5             = twom_x * (SEq[1]*SEq[2] - SEq[0]*SEq[3]) + 
                          twom_z * (SEq[0]*SEq[1] + SEq[2]*SEq[3]) - m[1];
  float f_6             = twom_x * (SEq_1SEq_3 + SEq_2SEq_4) + 
                          twom_z * (0.5 - SEq[1]*SEq[1] - SEq[2]*SEq[2]) - 
                          m[2];

  // compute the Jacobian
  float twom_xSEq_1     = 2.0f * m[0] * SEq[0];
  float twom_xSEq_2     = 2.0f * m[0] * SEq[1];
  float twom_xSEq_3     = 2.0f * m[0] * SEq[2];
  float twom_xSEq_4     = 2.0f * m[0] * SEq[3];
  float twom_zSEq_1     = 2.0f * m[2] * SEq[0];
  float twom_zSEq_2     = 2.0f * m[2] * SEq[1];
  float twom_zSEq_3     = 2.0f * m[2] * SEq[2];
  float twom_zSEq_4     = 2.0f * m[2] * SEq[3];
  float J_41            = twom_zSEq_3;
  float J_42            = twom_zSEq_4;
  float J_43            = 2.0f * twom_xSEq_3 + twom_zSEq_1; 
  float J_44            = 2.0f * twom_xSEq_4 - twom_zSEq_2;
  float J_51            = twom_xSEq_4 - twom_zSEq_2;
  float J_52            = twom_xSEq_3 + twom_zSEq_1;
  float J_53            = twom_xSEq_2 + twom_zSEq_4;
  float J_54            = twom_xSEq_1 - twom_zSEq_3;
  float J_61            = twom_xSEq_3;
  float J_62            = twom_xSEq_4 - 2.0f * twom_zSEq_2;
  float J_63            = twom_xSEq_1 - 2.0f * twom_zSEq_3;
  float J_64            = twom_xSEq_2;

  // calculate the gradient
  float SEqHatDot[4]    = {-J_41 * f_4 - J_51 * f_5 + J_61 * f_6,
                            J_42 * f_4 + J_52 * f_5 + J_62 * f_6,
                           -J_43 * f_4 + J_53 * f_5 + J_63 * f_6,
                           -J_44 * f_4 - J_54 * f_5 + J_64 * f_6};
  norm4(decrm(SEq, scale(norm4(SEqHatDot), config[id].mWeight)));
  FOM->delt             = SEqHatDot[0];
  
  // update state
  mutex_lock(id);
  memcpy(state[id].SEq, SEq, 4*sizeof(float));
  state[id].t           = mWeight * t + (1.0 - mWeight) * state[id].t;
  mutex_unlock(id);

  // exit function
  return 0;
}


/******************************************************************************
* estimate velocity vector (minus gravity)
******************************************************************************/

int IMU_core_estmQuat(
  uint16_t              id,
  float                 *estm)
{
  // copy current orientation state and return
  mutex_lock(id); 
  memcpy(estm, state[id].SEq, 4*sizeof(float));
  mutex_unlock(id);
  return 0;
}


/******************************************************************************
* estimate acceleration vector (minus gravity)
******************************************************************************/

int IMU_core_estmAccl(
  uint16_t              id,
  float                 *estm)
{
  // copy internal variables 
  mutex_lock(id); 
  float A[3];    memcpy(A,   state[id].A,   sizeof(A));
  float SEq[4];  memcpy(SEq, state[id].SEq, sizeof(SEq));
  mutex_unlock(id);

  // apply rotation to acceleration vector
  float tmp[4]          = {-SEq[1]*A[0] - SEq[2]*A[1] - SEq[3]*A[2],
                            SEq[0]*A[0] + SEq[2]*A[2] - SEq[3]*A[1],
                            SEq[0]*A[1] - SEq[1]*A[2] + SEq[3]*A[0],
                            SEq[0]*A[2] + SEq[1]*A[1] - SEq[2]*A[0]};
  estm[0] = -tmp[0]*SEq[1] + tmp[1]*SEq[0] - tmp[2]*SEq[3] + tmp[3]*SEq[2];
  estm[1] = -tmp[0]*SEq[2] + tmp[1]*SEq[3] + tmp[2]*SEq[0] - tmp[3]*SEq[1];
  estm[2] = -tmp[0]*SEq[3] - tmp[1]*SEq[2] + tmp[2]*SEq[1] + tmp[3]*SEq[0];
  
  return 0;
}


/******************************************************************************
* utility function - normalize 3x1 array
******************************************************************************/

inline float norm3(
  IMU_TYPE       *in, 
  float          *out)
{
  float norm     = sqrt((float)in[0]*(float)in[0] + 
                        (float)in[1]*(float)in[1] + 
                        (float)in[2]*(float)in[2]); 
  out[0]         = (float)in[0] / norm;
  out[1]         = (float)in[1] / norm;
  out[2]         = (float)in[2] / norm;
  return norm;
}


/******************************************************************************
* utility function - normalize 4x1 array
******************************************************************************/

inline float* norm4(
  float          *v)
{
  float norm     = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
  v[0]          /= norm;
  v[1]          /= norm;
  v[2]          /= norm;
  v[3]          /= norm;
  return v;     // allows function to be used as function argument
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


/******************************************************************************
* utility function - decrement 4x1 array by another 4x1 array
******************************************************************************/

inline float* decrm(
  float          *v, 
  float          *d)
{
  v[0]          -= d[0];
  v[1]          -= d[1];
  v[2]          -= d[2];
  v[3]          -= d[3];
  return v;
}

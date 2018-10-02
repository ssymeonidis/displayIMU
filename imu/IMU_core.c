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
#include "IMU_core.h"

// internally managed structures
struct IMU_core_config  config [IMU_MAX_INST]; 
struct IMU_core_state   state  [IMU_MAX_INST];
static unsigned short   IMU_core_inst = 0;


/******************************************************************************
* utility function - normalize 3x1 array
******************************************************************************/

inline float norm3(float* in, float* out)
{
  float norm    = sqrt(in[0]*in[0] + in[1]*in[1] + in[2]*in[2]); 
  out[0]        = in[0] / norm;
  out[1]        = in[1] / norm;
  out[2]        = in[2] / norm;
  return norm;
}


/******************************************************************************
* utility function - normalize 4x1 array
******************************************************************************/

inline float* norm4(float* v)
{
  float norm    = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
  v[0]         /= norm;
  v[1]         /= norm;
  v[2]         /= norm;
  v[3]         /= norm;
  return v;    // allows function to be used as function argument
}


/******************************************************************************
* utility function - scale 4x1 array by scalar value
******************************************************************************/

inline float* scale(float* v, float m)
{
  v[0]         *= m;
  v[1]         *= m;
  v[2]         *= m;
  v[3]         *= m;
  return v;    // allows function to be used as function argument
}


/******************************************************************************
* utility function - decrement 4x1 array by another 4x1 array
******************************************************************************/

inline float* decrm(float* v, float* d)
{
  v[0]         -= d[0];
  v[1]         -= d[1];
  v[2]         -= d[2];
  v[3]         -= d[3];
  return v;
}


/******************************************************************************
* utility function - get up component from quaternion
* (this function needs to be double checked)
******************************************************************************/

inline void getUpVector(unsigned short id, float* v)
{
  float* SEq    = state[id].SEq;
  float g[4]    = {-SEq[3] * config[id].aMag,  -SEq[2] * config[id].aMag,
                    SEq[1] * config[id].aMag,   SEq[0] * config[id].aMag};
  v[0]          =  -g[0]*SEq[1] + g[1]*SEq[0] + g[2]*SEq[3] - g[3]*SEq[2];
  v[1]          =  -g[0]*SEq[2] - g[1]*SEq[3] + g[2]*SEq[0] + g[3]*SEq[1];
  v[2]          =  -g[0]*SEq[3] + g[1]*SEq[2] - g[2]*SEq[1] + g[3]*SEq[0];
}


/******************************************************************************
* utility function - get forward component from quaternion
* (this function needs to be written)
******************************************************************************/

inline void getForwardVector(unsigned short id, float* v)
{
  v[0]          =  0;
  v[1]          =  0;
  v[2]          =  0;
}


/******************************************************************************
* function to return config structure handle
******************************************************************************/

int IMU_core_init(
  unsigned short              *id, 
  struct IMU_core_config      **pntr)
{
  // check for device count overflow
  if (IMU_core_inst >= IMU_MAX_INST)
    return IMU_CORE_INST_OVERFLOW;

  // return handle and calib pointer
  *id   = IMU_core_inst; 
  *pntr = &config[*id];
  IMU_core_inst++;
  return 0;
}


/******************************************************************************
* function to copy state structure
******************************************************************************/

void IMU_core_getState( 
  unsigned short              id,  
  struct IMU_core_state       **pntr)
{
  *pntr = &state[id];
}


/******************************************************************************
* initialize state and autocal to known state
******************************************************************************/

void IMU_core_reset(
  unsigned short              id)
{
  // initialize state to known value
  state[id].SEq[0]            = 1.0;
  state[id].SEq[1]            = 0.0;
  state[id].SEq[2]            = 0.0;
  state[id].SEq[3]            = 0.0;
  state[id].aReset            = config[id].isAccl;
  state[id].mReset            = config[id].isMagn;
}


/******************************************************************************
* core function for dead recon
* assumes: corrected and normalized data
******************************************************************************/

float* IMU_core_deadRecon(
  unsigned short              id,  
  float                       t,
  float                       *a_in, 
  float                       *m_in)
{
  // define the local variables
  float* SEq     = state[id].SEq;
  float  a[3];
  float  m[3];
  float  n;
  
  // normalize accerometer data or derive from system state
  if (!config[id].isAccl || a_in == NULL) {
    getUpVector(id, a);
  } else {
    norm3(a_in, a);
    state[id].aReset = 0;
  }
  
  // normalize magnetometer data or derive from system state
  if (!config[id].isMagn || m == NULL) {
    getForwardVector(id, m);
  } else {
    norm3(m_in, m);
    state[id].mReset = 0;
  }
  
  // ortho-normalize forwared vector 
  n            = m[0]*a[0]+m[1]*a[1]+m[2]*a[2];
  float f[3]   = {m[0]-n*a[0], 
                  m[1]-n*a[1], 
                  m[2]-n*a[2]};
  norm3(f, f);

  // calcuate the right vector
  float r[3]   = {a[1]*f[2]-a[2]*f[1], 
                  a[2]*f[0]-a[0]*f[2], 
                  a[0]*f[1]-a[1]*f[0]};

  // calculate the quaternion
  n            = f[0]+r[1]+a[2];
  if (n > 0) {
    n          = sqrt(1.0+n)*2;
    SEq[0]     = 0.25*n;
    SEq[1]     = (a[1]-r[2])/n;
    SEq[2]	 = (f[2]-a[0])/n;
    SEq[3]     = (r[0]-f[1])/n;
  } else if (f[0] > r[1] && f[0] > a[2]) {
    n          = sqrt(1.0+f[0]-r[1]-a[2])*2;
    SEq[0]     = (a[1]-r[2])/n;
    SEq[1]     = 0.25*n;
    SEq[2]     = (f[1]+r[0])/n;
    SEq[3]     = (f[2]+a[0])/n;
  } else if (r[1] > a[2]) {
    n          = sqrt(1.0+r[1]-f[0]-a[2])*2;
    SEq[0]     = (f[2]-a[0])/n;
    SEq[1]     = (f[1]+r[0])/n;
    SEq[2]     = 0.25*n;
    SEq[3]     = (r[2]+a[1])/n;
  } else {
    n          = sqrt(1.0+a[2]-f[0]-r[1])*2;
    SEq[0]     = (r[0]-f[1])/n;
    SEq[1]     = (f[2]+a[0])/n;
    SEq[2]     = (r[2]+a[1])/n;
    SEq[3]     = 0.25*n;
  }

  // exit function
  state[id].t  = t;
  return SEq;
}


/******************************************************************************
* apply gyroscope rates
******************************************************************************/

float* IMU_core_estmGyro(
  unsigned short               id, 
  float                        t, 
  float                        *g, 
  struct IMU_core_FOM          *FOM)
{
  // initialize figure of merit
  if (FOM != NULL) {
    FOM->type                  = gyro;
    FOM->data.gyro.stable      = 0;
  }
  
  // determine whether the function needs to be executed
  if (!config[id].isGyro || state[id].aReset || state[id].mReset)
    return state[id].SEq;
  
  // define internal variables
  unsigned char stable         = 0; 

  // update state last time
  state[id].t                  = t;

  // check for stable condition
  if (config[id].isStable) {
    float g_sum                = g[0] + g[1] + g[2];
    if (g_sum > 3 * config[id].gThreshVal)
      state[id].t_stable       = t;
    else if (t - state[id].t_stable > config[id].gThreshVal) {
      stable                   = 1;
      if (FOM != NULL)
        FOM->data.gyro.stable  = stable;
      return state[id].SEq;
    }
  }

  // define internal variables
  float half_dt                = 0.5 * (t - state[id].t);
  float *SEq                   = state[id].SEq;

  // compute gyro quaternion rate and apply delta to estimated orientation
  SEq[0]       += half_dt * (-SEq[1]*g[0] - SEq[2]*g[1] - SEq[3]*g[2]);
  SEq[1]       += half_dt * ( SEq[0]*g[0] + SEq[2]*g[2] - SEq[3]*g[1]);
  SEq[2]       += half_dt * ( SEq[0]*g[1] - SEq[1]*g[2] + SEq[3]*g[0]);
  SEq[3]       += half_dt * ( SEq[0]*g[2] + SEq[1]*g[1] - SEq[2]*g[0]);
  
  // exit function
  return SEq;
}


/******************************************************************************
* apply accelerometer vector
******************************************************************************/

float* IMU_core_estmAccl(
  unsigned short          id, 
  float                   t, 
  float                   *a_in, 
  struct IMU_core_FOM     *FOM)
{
  // save accelerometer data
  if (config[id].isMove) {
    state[id].a[0]        = a_in[0];
    state[id].a[1]        = a_in[1];
    state[id].a[2]        = a_in[2];
  }

  // initialize figure of merit
  if (FOM != NULL) {
    FOM->type             = accl;
    FOM->data.accl.aMag   = 0;
    FOM->data.accl.aDelt  = 0;
  }
  
  // determine whether the functions needs to be executed
  if (!config[id].isAccl)
    return state[id].SEq;
  if (state[id].aReset)
    return IMU_core_deadRecon(id, t, a_in, NULL);

  // define internal variables
  float *SEq              = state[id].SEq;
  float aMagFOM           = 0.0;
  float a[3];
    
  // normalize input vector
  float mag               = norm3(a_in, a);
  
  // determine datum quality factor
  float aWeight           = config[id].aWeight;
  if (config[id].isFOM) {
    float error           = abs(mag - config[id].aMag) / mag;
    aMagFOM               = 1.0 - error / config[id].aMagThresh;
    aMagFOM               = (aMagFOM < 0.0) ? 0.0 : aMagFOM;
    aWeight              *= aMagFOM;
  }
  if (FOM != NULL)
    FOM->data.accl.aMag   = aMagFOM;
  
  // check reset and zero weight condition
  if (aWeight <= 0.0)
    return SEq;

  // compute the objective function 
  float twoSEq[4]    = {2.0f*SEq[0], 2.0f*SEq[1], 2.0f*SEq[2], 2.0f*SEq[3]};
  float f_1          = twoSEq[1]*SEq[3] - twoSEq[0]*SEq[2] - a[0];
  float f_2          = twoSEq[0]*SEq[1] + twoSEq[2]*SEq[3] - a[1];
  float f_3          = 1.0f - twoSEq[1]*SEq[1] - twoSEq[2]*SEq[2] - a[2];
 
  // compute the Jacobian
  float J_11or24     = twoSEq[2]; 
  float J_12or23     = twoSEq[3];
  float J_13or22     = twoSEq[0];
  float J_14or21     = twoSEq[1];
  float J_32         = 2.0f * J_14or21;
  float J_33         = 2.0f * J_11or24;

  // calculate the gradient
  float SEqHatDot[4] = {J_14or21 * f_2 - J_11or24 * f_1,
                        J_12or23 * f_1 + J_13or22 * f_2 - J_32     * f_3,
                        J_12or23 * f_2 - J_33     * f_3 - J_13or22 * f_1,
                        J_14or21 * f_1 + J_11or24 * f_2};
  norm4(decrm(SEq, scale(norm4(SEqHatDot), aWeight)));
  if (FOM != NULL)
    FOM->data.accl.aDelt  = SEqHatDot[0];
  
  // exit function
  state[id].t             = aWeight * t + (1.0 - aWeight) * state[id].t;
  return SEq;
}


/******************************************************************************
* apply magnetometer vector
******************************************************************************/

float* IMU_core_estmMagn(
  unsigned short         id, 
  float                  t, 
  float                  *m_in, 
  struct IMU_core_FOM    *FOM)
{
  // initialize figure of merit
  if (FOM != NULL) {
    FOM->type            = magn;
    FOM->data.magn.mMag  = 0;
    FOM->data.magn.mAng  = 0;
    FOM->data.magn.mDelt = 0;
  }
  
  // determine whether the functions needs to be executed
  if (!config[id].isMagn)
    return state[id].SEq;
  if (state[id].mReset)
    return IMU_core_deadRecon(id, t, NULL, m_in);

  // define internal variables
  float *SEq             = state[id].SEq;
  float mMagFOM          = 0.0;
  float mAngFOM          = 0.0;
  float m[3];

  // normalize input vector
  float mag              = norm3(m_in, m);

  // determine datum quality factor
  float mWeight          = config[id].mWeight;
  if (config[id].isFOM) {
    // determine magnitude error
    float error          = abs(mag - config[id].mMag) / config[id].mMag;
    mMagFOM              = 1.0 - error / config[id].mMagThresh;
    mMagFOM              = (mMagFOM < 0.0) ? 0.0 : mMagFOM;
    mWeight             *= mMagFOM;
    
    // determine angle error
    float a[3];
    float ang;
    getUpVector(id, a);
    ang                  = acos(a[0]*m[0] + a[1]*m[1] + a[2]*m[2]);
    error                = abs(ang - config[id].mAng) / config[id].mAng;
    mAngFOM              = 1.0 - error / config[id].mAngThresh;
    mAngFOM              = (mAngFOM < 0.0) ? 0.0 : mAngFOM;
    mWeight             *= mAngFOM;
  }

  // update figure of merit
  if (FOM != NULL) {
    FOM->data.magn.mMag  = mMagFOM;
    FOM->data.magn.mAng  = mAngFOM;
  }
  
  // check zero weight conditiond
  if (mWeight <= 0.0)
    return SEq;

  // compute the objective function 
  float twom_x       = 2.0f * m[0];
  float twom_z       = 2.0f * m[2];
  float SEq_1SEq_3   = SEq[0] * SEq[2];
  float SEq_2SEq_4   = SEq[1] * SEq[3];
  float f_4          = twom_x * (0.5f - SEq[2]*SEq[2] - SEq[3]*SEq[3]) + 
                       twom_z * (SEq_2SEq_4 - SEq_1SEq_3) - m[0];
  float f_5          = twom_x * (SEq[1]*SEq[2] - SEq[0]*SEq[3]) + 
                       twom_z * (SEq[0]*SEq[1] + SEq[2]*SEq[3]) - m[1];
  float f_6          = twom_x * (SEq_1SEq_3 + SEq_2SEq_4) + 
                       twom_z * (0.5f - SEq[1]*SEq[1] - SEq[2]*SEq[2]) - m[2];

  // compute the Jacobian
  float twom_xSEq_1  = 2.0f * m[0] * SEq[0];
  float twom_xSEq_2  = 2.0f * m[0] * SEq[1];
  float twom_xSEq_3  = 2.0f * m[0] * SEq[2];
  float twom_xSEq_4  = 2.0f * m[0] * SEq[3];
  float twom_zSEq_1  = 2.0f * m[2] * SEq[0];
  float twom_zSEq_2  = 2.0f * m[2] * SEq[1];
  float twom_zSEq_3  = 2.0f * m[2] * SEq[2];
  float twom_zSEq_4  = 2.0f * m[2] * SEq[3];
  float J_41         = twom_zSEq_3;
  float J_42         = twom_zSEq_4;
  float J_43         = 2.0f * twom_xSEq_3 + twom_zSEq_1; 
  float J_44         = 2.0f * twom_xSEq_4 - twom_zSEq_2;
  float J_51         = twom_xSEq_4 - twom_zSEq_2;
  float J_52         = twom_xSEq_3 + twom_zSEq_1;
  float J_53         = twom_xSEq_2 + twom_zSEq_4;
  float J_54         = twom_xSEq_1 - twom_zSEq_3;
  float J_61         = twom_xSEq_3;
  float J_62         = twom_xSEq_4 - 2.0f * twom_zSEq_2;
  float J_63         = twom_xSEq_1 - 2.0f * twom_zSEq_3;
  float J_64         = twom_xSEq_2;

  // calculate the gradient
  float SEqHatDot[4] = {-J_41 * f_4 - J_51 * f_5 + J_61 * f_6,
                         J_42 * f_4 + J_52 * f_5 + J_62 * f_6,
                        -J_43 * f_4 + J_53 * f_5 + J_63 * f_6,
                        -J_44 * f_4 - J_54 * f_5 + J_64 * f_6};
  norm4(decrm(SEq, scale(norm4(SEqHatDot), config[id].mWeight)));
  if (FOM != NULL)
    FOM->data.magn.mDelt  = SEqHatDot[0];
  
  // exit function
  state[id].t             = mWeight * t + (1.0 - mWeight) * state[id].t;
  return SEq;
}


/******************************************************************************
* estimate euler angle given gyroscope, accelerometer, and magnetometer data 
******************************************************************************/

float* IMU_core_estmAll(
  unsigned short              id, 
  float                       t, 
  float                       *g, 
  float                       *a, 
  float                       *m,  
  struct IMU_core_FOM         FOM[3])
{
  // check for reset conditions
  if (state[id].mReset && state[id].aReset) {
    // initialize to known value
    if (FOM != NULL) {
      FOM[0].type             = gyro;
      FOM[0].data.gyro.stable = 0.0;
      FOM[1].type             = accl;
      FOM[1].data.accl.aMag   = 0.0;
      FOM[1].data.accl.aDelt  = 0.0;
      FOM[2].type             = magn;
      FOM[2].data.magn.mMag   = 0.0;
      FOM[2].data.magn.mAng   = 0.0;
      FOM[2].data.magn.mDelt  = 0.0;
    }
    
    // zero the system to the current sensor
    return IMU_core_deadRecon(id, t, a, m);
  }
  
  // update system state w/ each sensor
  IMU_core_estmGyro(id, t, g, &FOM[0]);
  IMU_core_estmAccl(id, t, a, &FOM[1]);
  IMU_core_estmMagn(id, t, m, &FOM[2]);
  return state[id].SEq;
}


/******************************************************************************
* estimate velocity vector (minus gravity)
******************************************************************************/

float* IMU_core_estmMove(unsigned short id)
{
  // define internal varirables
  float *A         = state[id].A;
  float *A_rot     = state[id].A_rot;
  float *a         = state[id].a;
  float *SEq       = state[id].SEq;
  float G[3];

  // rotate gravity up vector by estimated orientation
  getUpVector(id, G);
 
  // remove the gravity from the accelerometer data
  float alpha      = config[id].moveAlpha;
  if (state[id].aReset || state[id].mReset) {
    A[0]           = a[0]-G[0];
    A[1]           = a[1]-G[1];
    A[2]           = a[2]-G[2];
  } else {
    A[0]           = alpha*A[0] + (1.0f-alpha)*(a[0]-G[0]);
    A[1]           = alpha*A[1] + (1.0f-alpha)*(a[1]-G[1]);
    A[2]           = alpha*A[2] + (1.0f-alpha)*(a[2]-G[2]); 
  }
  
  // apply rotation to acceleration vector
  float tmp[4]     = {-SEq[1]*A[0] - SEq[2]*A[1] - SEq[3]*A[2],
                       SEq[0]*A[0] + SEq[2]*A[2] - SEq[3]*A[1],
                       SEq[0]*A[1] - SEq[1]*A[2] + SEq[3]*A[0],
                       SEq[0]*A[2] + SEq[1]*A[1] - SEq[2]*A[0]};
  A_rot[0] = -tmp[0]*SEq[1] + tmp[1]*SEq[0] - tmp[2]*SEq[3] + tmp[3]*SEq[2];
  A_rot[1] = -tmp[0]*SEq[2] + tmp[1]*SEq[3] + tmp[2]*SEq[0] - tmp[3]*SEq[1];
  A_rot[2] = -tmp[0]*SEq[3] - tmp[1]*SEq[2] + tmp[2]*SEq[1] + tmp[3]*SEq[0];
  return A_rot;
}

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

// include statements 
#include <math.h>            // sqrt/trig
#include <string.h>          // memcopy
#include "IMU.h"

// internally managed structures
displayIMU_calib     calib;
displayIMU_config    config; 
displayIMU_autocal   autocal;
displayIMU_state     state;


/******************************************************************************
* utility function - normalize 3x1 array
******************************************************************************/

inline float* norm3(float* v)
{
  float norm    = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); 
  v[0]         /= norm;
  v[1]         /= norm;
  v[2]         /= norm;
  return v;    // return allows function to be used as function argument
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
  return v;    // return allows function to be used as function argument
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
  return v;    // return allows function to be used as function argument
}


/******************************************************************************
* utility function - decrement 4x1 array by another 4x1 array
******************************************************************************/

inline void decrm(float* v, float* d)
{
  v[0]         -= d[0];
  v[1]         -= d[1];
  v[2]         -= d[2];
  v[3]         -= d[3];
}


/******************************************************************************
* function to return calib structure handle
******************************************************************************/

void displayIMU_getCalib(displayIMU_calib **calib_pntr) 
{
  *calib_pntr = &calib;
}


/******************************************************************************
* function to return config structure handle
******************************************************************************/

void displayIMU_getConfig(displayIMU_config **config_pntr) 
{
  *config_pntr = &config;
}


/******************************************************************************
* function to copy state structure
******************************************************************************/

void displayIMU_getState(displayIMU_state **state_pntr) 
{
  *state_pntr = &state;
}


/******************************************************************************
* function to copy autocal structure 
******************************************************************************/

void displayIMU_getAutocal(displayIMU_autocal *autocal_pntr) 
{
  memcpy(autocal_pntr, &autocal, sizeof(displayIMU_autocal));
}


/******************************************************************************
* set reference using last system quaternion
******************************************************************************/

void displayIMU_setRef()
{
  state.ref[0]       =  state.SEq[0];
  state.ref[1]       = -state.SEq[1];
  state.ref[2]       = -state.SEq[2];
  state.ref[3]       = -state.SEq[3];
}


/******************************************************************************
* set reference using up vector
* assumes: normalized input
******************************************************************************/

void displayIMU_setRefAccl(float* up)
{
  // updating reference based on accelerometer
  float* ref         = state.ref;
  float  tmp         = sqrt(2.0 + 2.0 * up[2]);
  if (tmp > 0.001) {
    ref[0]      =  0.5   * tmp;
    ref[1]      = -up[1] / tmp;
    ref[2]      =  up[0] / tmp;
    ref[3]      =  0.0;
  } else {
    ref[0]      =  0.0;
    ref[1]      =  0.0;
    ref[2]      =  1.0;
    ref[3]      =  0.0;
  }
}


/******************************************************************************
* correct raw gyroscope data
******************************************************************************/

void displayIMU_corGyro(float* g_raw, float* g)
{
  g[0]      = (g_raw[0] - calib.gBias[0]) / calib.gScale[0];
  g[1]      = (g_raw[1] - calib.gBias[1]) / calib.gScale[1];
  g[2]      = (g_raw[2] - calib.gBias[2]) / calib.gScale[2];
}


/******************************************************************************
* correct raw accelerometer data
******************************************************************************/

void displayIMU_corAccl(float* a_raw, float* a)
{
  a[0]      = a_raw[0] - calib.aBias[0];
  a[1]      = a_raw[1] - calib.aBias[1];
  a[2]      = a_raw[2] - calib.aBias[2];
}


/******************************************************************************
* correct raw magnetometer data
******************************************************************************/

void displayIMU_corMagn(float* m_raw, float* m)
{
  m[0]      = m_raw[0] - calib.mBias[0];
  m[1]      = m_raw[1] - calib.mBias[1];
  m[2]      = m_raw[2] - calib.mBias[2];
}


/******************************************************************************
* correct raw gyroscope, accelerometer, and magnetometer data
******************************************************************************/

void displayIMU_corAll(float* g_raw, float* a_raw, float* m_raw,
                       float* g,     float* a,     float* m)
{
  displayIMU_corGyro(g_raw, g);
  displayIMU_corAccl(a_raw, a);
  displayIMU_corMagn(m_raw, m);
}


/******************************************************************************
* initialize state and autocal to known state
******************************************************************************/

void displayIMU_init()
{
  // initialize state to known value
  state.SEq[0]      = 1.0;
  state.SEq[1]      = 0.0;
  state.SEq[2]      = 0.0;
  state.SEq[3]      = 0.0;
  state.ref[0]      = 1.0;
  state.ref[1]      = 0.0;
  state.ref[2]      = 0.0;
  state.ref[3]      = 0.0;
  state.isReset     = true;

  // initialize autocal structure w/ previous value
  memcpy(autocal.gBias,     calib.gBias, 3*sizeof(float));
  memcpy(autocal.gBiasCont, calib.gBias, 3*sizeof(float));
  autocal.aMag      = calib.aMag;
  autocal.aMagCont  = calib.aMag;
  autocal.mMag      = calib.mMag;
  autocal.mMagCont  = calib.mMag;
  autocal.mAng      = calib.mAng;
  autocal.mAngCont  = calib.mAng;
}


/******************************************************************************
* core function for dead recon
* assumes: corrected and normalized data
******************************************************************************/

inline float* displayIMU_updateAbs(float* a, float* m)
{
  // define the local variables
  float* SEq         = state.SEq;
  float  n;
  
  // update the system quaternian
  if (config.isAccl != 0 && config.isMagn != 0 && a != NULL && m != NULL) {

    // ortho-normalize forwared vector 
    n            = m[0]*a[0]+m[1]*a[1]+m[2]*a[2];
    float f[3]   = {m[0]-n*a[0], 
                    m[1]-n*a[1], 
		    m[2]-n*a[2]};
    norm3(f);

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
  } else if (config.isAccl != 0 && a != NULL) {
    n            =  sqrt(2.0 + 2.0 * a[2]);
    SEq[0]       =  0.5  * n;
    SEq[1]       =  a[1] / n;
    SEq[2]       = -a[0] / n;
    SEq[3]       =  0.0;
  } else if (config.isMagn != 0 && m != NULL) {
    n            =  sqrt(2.0 + 2.0 * m[0]);
    SEq[0]       =  0.5 * n;
    SEq[1]       =  0.0;
    SEq[2]       =  m[2] / n;
    SEq[3]       = -m[1] / n;
  }
  
  // exit function
  return SEq;
}


/******************************************************************************
* apply gyroscope rates
* assumes: corrected and normalized data
******************************************************************************/

inline float* displayIMU_updateGyro(float* g)
{
  // determine whether the function needs to be executed
  if (config.isGyro == false)
    return state.SEq;

  // define internal variables
  float halfSEq[4] = {0.5f*state.SEq[0], 0.5f*state.SEq[1],
                      0.5f*state.SEq[2], 0.5f*state.SEq[3]};

  // compute gyro quaternion rate and apply delta to estimated orientation 
  state.SEq[0]    += -halfSEq[1]*g[0] - halfSEq[2]*g[1] - halfSEq[3]*g[2];
  state.SEq[1]    +=  halfSEq[0]*g[0] + halfSEq[2]*g[2] - halfSEq[3]*g[1];
  state.SEq[2]    +=  halfSEq[0]*g[1] - halfSEq[1]*g[2] + halfSEq[3]*g[0];
  state.SEq[3]    +=  halfSEq[0]*g[2] + halfSEq[1]*g[1] - halfSEq[2]*g[0];
  
  // exit function
  return state.SEq;
}


/******************************************************************************
* apply accelerometer vector
* assumes: corrected and normalized data
******************************************************************************/

inline float* displayIMU_updateAccl(float* a)
{
  // determine whether the functions needs to be executed
  if (config.isAccl == false)
    return state.SEq;

  // define internal variables
  float*  SEq        = state.SEq;
  float   twoSEq[4]  = {2.0f*SEq[0], 2.0f*SEq[1], 2.0f*SEq[2], 2.0f*SEq[3]};

  // compute the objective function 
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
  decrm(SEq, scale(norm4(SEqHatDot), config.mWeight));

  // calculate delta metric
  // delta_a            = SEqHatDot[0];
  
  // exit function
  return SEq;
}


/******************************************************************************
* apply magnetometer vector
* assumes: corrected and normalized data
******************************************************************************/

inline float* displayIMU_updateMagn(float* m)
{
  // determine whether the functions needs to be executed
  if (config.isMagn == false)
    return state.SEq;

  // define internal variables
  float*  SEq        = state.SEq;

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
  decrm(SEq, scale(norm4(SEqHatDot), config.mWeight));

  /*
  // calculate delta metric
  delta_m            = SEqHatDot[0];
  
  // calcuate the magnitude error
  norm           = sqrt(m_x*m_x + m_y*m_y + m_z*m_z);
  norm           = 100*(norm-calib.mMag)/calib.mMag;
  delta_M        = error_fltr*delta_M + (1-error_fltr)*norm; 

  // calcuate the angle (minus the arccos function)
  norm           =  (a_x*m_x + a_y*m_y + a_z*m_z) /
                    (sqrt(a_x*a_x + a_y*a_y + a_z*a_z) *
                     sqrt(m_x*m_x + m_y*m_y + m_z*m_z));
  norm           = 100*(norm-calib.mAng)/calib.mAng;
  delta_ang      = error_fltr*delta_ang + (1-error_fltr)*norm; 
  */

  // exit function
  return SEq;
}


/******************************************************************************
* calculate euler angle
* assumes: corrected and normalized data
******************************************************************************/

inline void displayIMU_applyRef(float* q, float* ref, float* q_out)
{
  q_out[0] = q[0]*ref[0] - q[1]*ref[1] - q[2]*ref[2] - q[3]*ref[3];
  q_out[1] = q[0]*ref[1] + q[1]*ref[0] + q[2]*ref[3] - q[3]*ref[2];
  q_out[2] = q[0]*ref[2] - q[1]*ref[3] + q[2]*ref[0] + q[3]*ref[1];
  q_out[3] = q[0]*ref[3] + q[1]*ref[2] - q[2]*ref[1] + q[3]*ref[0];
}


/******************************************************************************
* calculate euler angle
******************************************************************************/

inline void displayIMU_calcEuler(float* q, float* E)
{
  float Q[4] = {q[0]*q[0], q[1]*q[1], q[2]*q[2], q[3]*q[3]};
  E[0] = 180 * atan2(2*(q[1]*q[3]+q[3]*q[0]),  Q[1]-Q[2]-Q[3]+Q[0]) / M_PI;
  E[1] = 180 * asin(-2*(q[1]*q[3]-q[2]*q[0])) / M_PI;
  E[2] = 180 * atan2(2*(q[2]*q[3]+q[1]*q[0]), -Q[1]-Q[2]+Q[3]+Q[0]) / M_PI;
}


/******************************************************************************
* calculate euler angle
******************************************************************************/

inline void displayIMU_refAndEuler(float* E)
{
  if (config.isTear == true){
    float q_tmp[4];
    displayIMU_applyRef(state.SEq, state.ref, q_tmp);
    displayIMU_calcEuler(q_tmp, E);
  } else {
    displayIMU_calcEuler(state.SEq, E);
  }
}


/******************************************************************************
* estimate velocity vector (minus gravity)
******************************************************************************/

inline void displayIMU_estmAccl(float* a, float* A)
{
  if (config.isAcclEstm == false)
    return;

  // define internal varirables
  float* SEq       = state.SEq;

  // rotate gravity up vector by estimated orientation
  float g[4] = {-SEq[3] * calib.aMag,  -SEq[2] * calib.aMag,
                 SEq[1] * calib.aMag,   SEq[0] * calib.aMag};
  float G[3] = {-g[0]*SEq[1] + g[1]*SEq[0] + g[2]*SEq[3] - g[3]*SEq[2],
                -g[0]*SEq[2] - g[1]*SEq[3] + g[2]*SEq[0] + g[3]*SEq[1],
                -g[0]*SEq[3] + g[1]*SEq[2] - g[2]*SEq[1] + g[3]*SEq[0]};
 
  // remove the gravity from the accelerometer data
  float alpha      = config.acclAlpha;
  if (state.isReset != 0) {
    A[0]           = a[0]-G[0];
    A[1]           = a[1]-G[1];
    A[2]           = a[2]-G[2];
  } else {
    A[0]           = alpha*state.A[0] + (1.0f-alpha)*(a[0]-G[0]);
    A[1]           = alpha*state.A[1] + (1.0f-alpha)*(a[1]-G[1]);
    A[2]           = alpha*state.A[2] + (1.0f-alpha)*(a[2]-G[2]); 
  }
  
  // save acceleration estimate to system state
  state.A[0]       = A[0];
  state.A[1]       = A[1];
  state.A[2]       = A[2];

  // apply rotation to acceleration vector
  float tmp[4]     = {-SEq[1]*A[0] - SEq[2]*A[1] - SEq[3]*A[2],
                       SEq[0]*A[0] + SEq[2]*A[2] - SEq[3]*A[1],
                       SEq[0]*A[1] - SEq[1]*A[2] + SEq[3]*A[0],
                       SEq[0]*A[2] + SEq[1]*A[1] - SEq[2]*A[0]};
  A[0] = -tmp[0]*SEq[1] + tmp[1]*SEq[0] - tmp[2]*SEq[3] + tmp[3]*SEq[2];
  A[1] = -tmp[0]*SEq[2] + tmp[1]*SEq[3] + tmp[2]*SEq[0] - tmp[3]*SEq[1];
  A[2] = -tmp[0]*SEq[3] - tmp[1]*SEq[2] + tmp[2]*SEq[1] + tmp[3]*SEq[0];
}


/******************************************************************************
* estimate euler angle given acclerometer and magnetomer (no filtering)
******************************************************************************/

void displayIMU_deadRecon(float* a, float* m, float* E)
{
  // update system state (quaternion)
  displayIMU_updateAbs(norm3(a), norm3(m));
  displayIMU_refAndEuler(E);
  state.isReset = false;
}


/******************************************************************************
* estimate euler angle given gyroscope data (asynchronous operation)
******************************************************************************/

void  displayIMU_estmGyro(float* t, float* g, float* E, float* A,
  displayIMU_metrics* FOM)
{
  if (state.isReset == true || config.isFltr == false)
    return;

  // derive euler and acceleration from system state
  norm4(displayIMU_updateGyro(norm3(g)));
  displayIMU_refAndEuler(E);
  displayIMU_estmAccl(state.a, A);
}


/******************************************************************************
* estimate euler angle given magnetometer data (asynchronous operation)
******************************************************************************/

void  displayIMU_estmAccl(float* t,  float* a, float* E, float* A,
  displayIMU_metrics* FOM)
{
  // save original acclerometer if estimating accleration
  if (config.isAccl == true)
    memcpy(state.a, a, 3*sizeof(float));
  
  // update system state (quaternion)
  if (state.isReset == true || config.isFltr == false) {
    displayIMU_updateAbs(norm3(a), NULL);
    state.isReset = false;
  } else {
    norm4(displayIMU_updateAccl(norm3(a)));
  }
  
  // derive euler and acceleration from system state
  displayIMU_refAndEuler(E);
  displayIMU_estmAccl(state.a, A);
}


/******************************************************************************
* estimate euler angle given gyroscope, magnetometer, and accelerometer
******************************************************************************/

void  displayIMU_estmMagn(float* t, float* m, float* E, float* A,
  displayIMU_metrics* FOM)
{
  // update system state (quaternion)
  if (state.isReset == true || config.isFltr == false) {
    displayIMU_updateAbs(NULL, norm3(m));
    state.isReset = false;
  } else {
    norm4(displayIMU_updateMagn(norm3(m)));
  }
  
  // derive euler and acceleration from system state
  displayIMU_refAndEuler(E);
  displayIMU_estmAccl(state.a, A);
}


/******************************************************************************
* estimate euler angle given gyroscope, accelerometer, and magnetometer data 
******************************************************************************/

void  displayIMU_estmAll(float* t, float* g, float* a, float* m, float* E, 
  float* A, displayIMU_metrics* FOM)
{
  // save original acclerometer if estimating accleration
  if (config.isAccl == true)
    memcpy(state.a, a, 3*sizeof(float));
  
  // update system state (quaternion)
  if (state.isReset == true || config.isFltr == false) {
    displayIMU_updateAbs(norm3(a), norm3(m));
    state.isReset = false;
  } else {
    displayIMU_updateGyro(norm3(g));
    displayIMU_updateAccl(norm3(a));
    displayIMU_updateMagn(norm3(m));
    norm4(state.SEq);
  }
  
  // derive euler and acceleration from system state
  displayIMU_refAndEuler(E);
  displayIMU_estmAccl(state.a, A);
}

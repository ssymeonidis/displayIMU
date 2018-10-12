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

// include statements 
#include <math.h>            // sqrt/trig
#include "IMU_math.h"



/******************************************************************************
* utility function - get up component from quaternion
* (this function needs to be double checked)
******************************************************************************/

float* IMU_math_quatToUp(
  float*                q, 
  float*                v)
{
  v[0]                  =  q[3]*q[1] - q[2]*q[0] + q[1]*q[3] - q[0]*q[2];
  v[1]                  =  q[3]*q[2] + q[2]*q[3] + q[1]*q[0] + q[0]*q[1];
  v[2]                  =  q[3]*q[3] - q[2]*q[2] - q[1]*q[1] + q[0]*q[0];
  return v;             // allows function to be used as function argument
}


/******************************************************************************
* utility function - get forward component from quaternion
* (this function needs to be written)
******************************************************************************/

float* IMU_math_quatToFrwd(
  float*                q, 
  float*                v)
{
  v[0]                  =  0;
  v[1]                  =  0;
  v[2]                  =  0;
  return v;             // allows function to be used as function argument
}


/******************************************************************************
* utility function - get quaternion from forward and up vectors
******************************************************************************/

float* IMU_math_upFrwdToQuat(
  float*                u,
  float*                f_in,
  float*                q)
{
  // ortho-normalize forwared vector 
  float n               = u[0]*f_in[0] + u[1]*f_in[1] + u[2]*f_in[2];
  float f[3]            = {f_in[0] - n*u[0], 
                           f_in[1] - n*u[1], 
                           f_in[2] - n*u[2]};
  n                     = sqrtf(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
  f[0]                  = f[0] / n;
  f[1]                  = f[1] / n;
  f[2]                  = f[2] / n;

  // calcuate the right vector
  float r[3]            = {u[1]*f[2] - u[2]*f[1], 
                           u[2]*f[0] - u[0]*f[2], 
                           u[0]*f[1] - u[1]*f[0]};

  // calculate the quaternion
  n                     = f[0]+r[1]+u[2];
  if (n > 0) {
    n                   = sqrtf(1.0+n)*2;
    q[0]                = 0.25*n;
    q[1]                = (u[1]-r[2])/n;
    q[2]                = (f[2]-u[0])/n;
    q[3]                = (r[0]-f[1])/n;
  } else if (f[0] > r[1] && f[0] > u[2]) {
    n                   = sqrtf(1.0+f[0]-r[1]-u[2])*2;
    q[0]                = (u[1]-r[2])/n;
    q[1]                = 0.25*n;
    q[2]                = (f[1]+r[0])/n;
    q[3]                = (f[2]+u[0])/n;
  } else if (r[1] > u[2]) {
    n                   = sqrtf(1.0+r[1]-f[0]-u[2])*2;
    q[0]                = (f[2]-u[0])/n;
    q[1]                = (f[1]+r[0])/n;
    q[2]                = 0.25*n;
    q[3]                = (r[2]+u[1])/n;
  } else {
    n                   = sqrtf(1.0+u[2]-f[0]-r[1])*2;
    q[0]                = (r[0]-f[1])/n;
    q[1]                = (f[2]+u[0])/n;
    q[2]                = (r[2]+u[1])/n;
    q[3]                = 0.25*n;
  }

  return q;      // allows function to be used as function argument
}


/******************************************************************************
* convert quaternion to Euler angle
******************************************************************************/

float* IMU_math_quatToEuler(
  float                 *q, 
  float                 *E)
{
  float Q[4] = {q[0]*q[0], q[1]*q[1], q[2]*q[2], q[3]*q[3]};
  E[0] = 180 * atan2f(2*(q[1]*q[3]+q[3]*q[0]),  Q[1]-Q[2]-Q[3]+Q[0]) / M_PI;
  E[1] = 180 * asinf(-2*(q[1]*q[3]-q[2]*q[0])) / M_PI;
  E[2] = 180 * atan2f(2*(q[2]*q[3]+q[1]*q[0]), -Q[1]-Q[2]+Q[3]+Q[0]) / M_PI;
  return E;      // allows function to be used as function argument
}


/******************************************************************************
* apply reference quaterion to current orientation
******************************************************************************/

float* IMU_math_applyRef(
  float                 *q, 
  float                 *ref, 
  float                 *q_out)
{
  q_out[0] = q[0]*ref[0] - q[1]*ref[1] - q[2]*ref[2] - q[3]*ref[3];
  q_out[1] = q[0]*ref[1] + q[1]*ref[0] + q[2]*ref[3] - q[3]*ref[2];
  q_out[2] = q[0]*ref[2] - q[1]*ref[3] + q[2]*ref[0] + q[3]*ref[1];
  q_out[3] = q[0]*ref[3] + q[1]*ref[2] - q[2]*ref[1] + q[3]*ref[0];
  return q_out;  // allows function to be used as function argument
}

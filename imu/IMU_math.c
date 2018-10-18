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
* convert quaternion to Euler angles
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
******************************************************************************/

float* IMU_math_quatToEuler(
  float                 *q, 
  float                 *E)
{
  // roll (x-axis rotation)
  float sinr_cosp = 2.0 * (q[0]*q[1] + q[2]*q[3]);
  float cosr_cosp = 1.0 - 2.0 * (q[1]*q[1] + q[2]*q[2]);
  E[2]  = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  float sinp = 2.0 * (q[0]*q[2] - q[3]*q[1]);
  if (fabs(sinp) >= 1)
    E[1] = copysign(M_PI/2, sinp);
  else
    E[1] = asin(sinp);

  // yaw (z-axis rotation)
  float siny_cosp = 2.0 * (q[0]*q[3] + q[1]*q[2]);
  float cosy_cosp = 1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3]);
  E[0]  = atan2(siny_cosp, cosy_cosp);
  
  // pass Euler angle
  return E; 
}


/******************************************************************************
* convert Euler angles to quaternion
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
******************************************************************************/

float* IMU_math_eulerToQuat(
  float                 *E, 
  float                 *q)
{
  // angle function abreviations
  float cy = cos(E[0] * 0.5);
  float sy = sin(E[0] * 0.5);
  float cr = cos(E[2] * 0.5);
  float sr = sin(E[2] * 0.5);
  float cp = cos(E[1] * 0.5);
  float sp = sin(E[1] * 0.5);

  // conversion to quaternion
  q[0] = cy*cr*cp + sy*sr*sp;
  q[1] = cy*sr*cp - sy*cr*sp;
  q[2] = cy*cr*sp + sy*sr*cp;
  q[3] = sy*cr*cp - cy*sr*sp;
  return q;  
}


/******************************************************************************
* convert radians to degrees
******************************************************************************/

float* IMU_math_radToDeg(
  float                 *r, 
  float                 *d)
{
  float scale = 180 / M_PI;
  d[0]        = scale * r[0];
  d[1]        = scale * r[1];
  d[2]        = scale * r[2];
  return d;
}


/******************************************************************************
* convert radians to degrees
******************************************************************************/

float* IMU_math_degToRad(
  float                 *d, 
  float                 *r)
{
  float scale = M_PI / 180;
  r[0]        = scale * d[0];
  r[1]        = scale * d[1];
  r[2]        = scale * d[2];
  return r;
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

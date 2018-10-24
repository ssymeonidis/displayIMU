/*
 * This file is part of quaternion-based displayIMU C/C/C++/QT code base
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

#ifndef _IMU_MATH_H
#define _IMU_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements 
#include <math.h>            // sqrt/trig

// basic quaternion operators
static inline float* IMU_math_quatMult      (float *q1, float *q2, float *out);
static inline float* IMU_math_quatMultConj  (float *q1, float *q2, float *out);
static inline float* IMU_math_rotateForward (float *v,  float *q,  float *out);
static inline float* IMU_math_rotateReverse (float *v,  float *q,  float *out);

// converting between quaternions and pointing vectors
static inline float* IMU_math_quatToUp      (float *q, float *v);
static inline float* IMU_math_quatToFrwd    (float *q, float *v);
static inline float* IMU_math_upToQuat      (float *u, float *q);
static inline float* IMU_math_upFrwdToQuat  (float *u, float *f,   float *q);
static inline float* IMU_math_vectToQuat    (float *u, float *v,   float *q);

/// converting between quaternions and Euler angles
static inline float* IMU_math_quatToEuler   (float *q, float *E);
static inline float* IMU_math_eulerToQuat   (float *E, float *q);
static inline float* IMU_math_radToDeg      (float *r, float *d);
static inline float* IMU_math_degToRad      (float *d, float *r);

// general utility functions
static inline float IMU_math_calcWeight (float val, float ref, float thresh);

// core filters
int    IMU_math_estmAccl      (float *q, float *a, float alpha, float *FOM);
int    IMU_math_estmMagnRef   (float *q, float *m, float refx,  float refz,
                               float alpha, float *FOM);


/******************************************************************************
* quaternion multiplication 
******************************************************************************/

inline float* IMU_math_quatMult(
  float                 *q1, 
  float                 *q2,
  float                 *out)
{
  out[0] = q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2] - q2[3]*q1[3];
  out[1] = q2[0]*q1[1] + q2[1]*q1[0] - q2[2]*q1[3] + q2[3]*q1[2];
  out[2] = q2[0]*q1[2] + q2[1]*q1[3] + q2[2]*q1[0] - q2[3]*q1[1];
  out[3] = q2[0]*q1[3] - q2[1]*q1[2] + q2[2]*q1[1] + q2[3]*q1[0];
  return out;  // allows function to be used as function argument
}


/******************************************************************************
* quaternion multiplication w/ conjugagte
******************************************************************************/

inline float* IMU_math_quatMultConj(
  float                 *q1, 
  float                 *q2,
  float                 *out)
{
  out[0] = q2[0]*q1[0] + q2[1]*q1[1] + q2[2]*q1[2] + q2[3]*q1[3];
  out[1] = q2[0]*q1[1] - q2[1]*q1[0] + q2[2]*q1[3] - q2[3]*q1[2];
  out[2] = q2[0]*q1[2] - q2[1]*q1[3] - q2[2]*q1[0] + q2[3]*q1[1];
  out[3] = q2[0]*q1[3] + q2[1]*q1[2] - q2[2]*q1[1] - q2[3]*q1[0];
  return out;  // allows function to be used as function argument
}


/******************************************************************************
* rotate vector by quaternion (forward)
******************************************************************************/

inline float* IMU_math_rotateForward(
  float                 *v, 
  float                 *q,
  float                 *out)
{
  out[0] = 2.0f * (v[0] * (0.5f - q[2]*q[2] - q[3]*q[3])
                 + v[1] * (q[1]*q[2] - q[0]*q[3]) 
                 + v[2] * (q[1]*q[3] + q[0]*q[2]));
  out[1] = 2.0f * (v[0] * (q[1]*q[2] + q[0]*q[3])
                 + v[1] * (0.5f - q[1]*q[1] - q[3]*q[3])
                 + v[2] * (q[2]*q[3] - q[0]*q[1]));
  out[2] = 2.0f * (v[0] * (q[1]*q[3] - q[0]*q[2])
                 + v[1] * (q[2]*q[3] + q[0]*q[1])
                 + v[2] * (0.5f - q[1]*q[1] - q[2]*q[2]));
  return out;
}


/******************************************************************************
* rotate vector by quaternion (reverse)
******************************************************************************/

inline float* IMU_math_rotateReverse(
  float                 *v, 
  float                 *q,
  float                 *out)
{
  out[0] = 2.0f * (v[0] * (0.5f - q[2]*q[2] - q[3]*q[3])
                 + v[1] * (q[1]*q[2] + q[0]*q[3]) 
                 + v[2] * (q[1]*q[3] - q[0]*q[2]));
  out[1] = 2.0f * (v[0] * (q[1]*q[2] - q[0]*q[3])
                 + v[1] * (0.5f - q[1]*q[1] - q[3]*q[3])
                 + v[2] * (q[2]*q[3] + q[0]*q[1]));
  out[2] = 2.0f * (v[0] * (q[1]*q[3] + q[0]*q[2])
                 + v[1] * (q[2]*q[3] - q[0]*q[1])
                 + v[2] * (0.5f - q[1]*q[1] - q[2]*q[2]));
  return out;
}


/******************************************************************************
* get up component from quaternion
******************************************************************************/

inline float* IMU_math_quatToUp(
  float                 *q,
  float                 *v)
{
  v[0]                  =  2.0f * (q[1]*q[3] + q[0]*q[2]);
  v[1]                  =  2.0f * (q[2]*q[3] - q[0]*q[1]);
  v[2]                  =  2.0f * (0.5f - q[1]*q[1] - q[2]*q[2]);
  return v;
}


/******************************************************************************
* get forward component from quaternion
******************************************************************************/

inline float* IMU_math_quatToFrwd(
  float                 *q, 
  float                 *v)
{
  v[0]                  =  2.0f * (0.5f - q[2]*q[2] - q[3]*q[3]);
  v[1]                  =  2.0f * (q[1]*q[2] + q[0]*q[3]);
  v[2]                  =  2.0f * (q[1]*q[3] - q[0]*q[2]);
  return v;
}


/******************************************************************************
* get up component from quaternion
******************************************************************************/

inline float* IMU_math_upToQuat(
  float                 *u,
  float                 *q)
{
  float norm            = sqrtf(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
  q[0]                  = norm + u[2];
  if (q[0] > 0.001f * norm) {
    norm                = sqrtf(q[0]*q[0] + u[0]*u[0] + u[1]*u[1]);
    q[0]                =  q[0] / norm;
    q[1]                = -u[1] / norm;
    q[2]                =  u[0] / norm;
    q[3]                =  0.0f;
  } else {
    q[0]                =  1.0f;
    q[1]                =  0.0f;
    q[2]                =  0.0f;
    q[3]                =  0.0f;
  }
  return q;
}


/******************************************************************************
* get quaternion from forward and up vectors
******************************************************************************/

inline float* IMU_math_upFrwdToQuat(
  float                 *u_in,
  float                 *f_in,
  float                 *q)
{
  // normalize up vector
  float n       = sqrtf(u_in[0]*u_in[0] + u_in[1]*u_in[1] + u_in[2]*u_in[2]);
  float u[3]    = {u_in[0]/n, u_in[1]/n, u_in[2]/n};


  // ortho-normalize forward vector 
  n             = u[0]*f_in[0] + u[1]*f_in[1] + u[2]*f_in[2];
  float f[3]    = {f_in[0] - n*u[0], f_in[1] - n*u[1], f_in[2] - n*u[2]};
  n             = sqrtf(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
  f[0]          = f[0] / n;
  f[1]          = f[1] / n;
  f[2]          = f[2] / n;

  // calcuate the right vector (cross product)
  float r[3]    = {u[1]*f[2] - u[2]*f[1],
                   u[2]*f[0] - u[0]*f[2],
                   u[0]*f[1] - u[1]*f[0]};

  // calculate the quaternion
  n             = f[0]+r[1]+u[2];
  if (n > 0) {
    n           = sqrtf(1.0+n)*2;
    q[0]        = 0.25f*n;
    q[1]        = (u[1]-r[2])/n;
    q[2]        = (f[2]-u[0])/n;
    q[3]        = (r[0]-f[1])/n;
  } else if (f[0] > r[1] && f[0] > u[2]) {
    n           = sqrtf(1.0+f[0]-r[1]-u[2])*2;
    q[0]        = (u[1]-r[2])/n;
    q[1]        = 0.25f*n;
    q[2]        = (f[1]+r[0])/n;
    q[3]        = (f[2]+u[0])/n;
  } else if (r[1] > u[2]) {
    n           = sqrtf(1.0+r[1]-f[0]-u[2])*2;
    q[0]        = (f[2]-u[0])/n;
    q[1]        = (f[1]+r[0])/n;
    q[2]        = 0.25f*n;
    q[3]        = (r[2]+u[1])/n;
  } else {
    n           = sqrtf(1.0+u[2]-f[0]-r[1])*2;
    q[0]        = (r[0]-f[1])/n;
    q[1]        = (f[2]+u[0])/n;
    q[2]        = (r[2]+u[1])/n;
    q[3]        = 0.25f*n;
  }

  // exit function (allows function to be used as function argument)
  return q;
}


/******************************************************************************
* get forward component from quaternion
******************************************************************************/

inline float* IMU_math_vectToQuat(
  float                 *u, 
  float                 *v,
  float                 *q)
{
  float norm            = sqrt ((u[0]*u[0] + u[1]*u[1] + u[2]*u[2]) *
                                (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
  q[0]                  = norm + u[0]*v[0] + u[1]*v[1] + u[2]*u[2];
  if (q[0] > 0.001f * norm) {
    q[1]                = u[1]*v[2] - u[2]*v[1];
    q[2]                = u[2]*v[0] - u[0]*v[2];
    q[3]                = u[0]*v[1] - u[1]*v[0];
  } else if (fabs(u[0]) > fabs(u[2])) {
    q[1]                = -u[1];
    q[2]                = u[0];
    q[3]                = 0.0f;
  } else {
    q[1]                = 0.0f;
    q[2]                = -u[2];
    q[3]                = u[1];
  }
  norm                  = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0]                  = q[0] / norm;
  q[1]                  = q[1] / norm;
  q[2]                  = q[2] / norm;
  q[3]                  = q[3] / norm;
  return q;
}


/******************************************************************************
* convert quaternion to Euler angles
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
******************************************************************************/

inline float* IMU_math_quatToEuler(
  float                 *q, 
  float                 *E)
{
  // roll (x-axis rotation)
  float sinr_cosp = 2.0f * (q[0]*q[1] + q[2]*q[3]);
  float cosr_cosp = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
  E[2]  = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  float sinp = 2.0f * (q[0]*q[2] - q[3]*q[1]);
  if (fabs(sinp) >= 1)
    E[1] = copysign(M_PI/2, sinp);
  else
    E[1] = asin(sinp);

  // yaw (z-axis rotation)
  float siny_cosp = 2.0f * (q[0]*q[3] + q[1]*q[2]);
  float cosy_cosp = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
  E[0]  = atan2(siny_cosp, cosy_cosp);
  
  // pass Euler angle
  return E; 
}


/******************************************************************************
* convert Euler angles to quaternion
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
******************************************************************************/

inline float* IMU_math_eulerToQuat(
  float                 *E, 
  float                 *q)
{
  // angle function abreviations
  float cy = cos(E[0] * 0.5f);
  float sy = sin(E[0] * 0.5f);
  float cr = cos(E[2] * 0.5f);
  float sr = sin(E[2] * 0.5f);
  float cp = cos(E[1] * 0.5f);
  float sp = sin(E[1] * 0.5f);

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

inline float* IMU_math_radToDeg(
  float                 *r, 
  float                 *d)
{
  float scale           = 180.0f / M_PI;
  d[0]                  = scale * r[0];
  d[1]                  = scale * r[1];
  d[2]                  = scale * r[2];
  return d;
}


/******************************************************************************
* convert radians to degrees
******************************************************************************/

inline float* IMU_math_degToRad(
  float                 *d, 
  float                 *r)
{
  float scale           = M_PI / 180.0f;
  r[0]                  = scale * d[0];
  r[1]                  = scale * d[1];
  r[2]                  = scale * d[2];
  return r;
}


/******************************************************************************
* function used to apply target and threshold to generate weight 
******************************************************************************/

static float IMU_math_calcWeight(
  float                 val, 
  float                 ref, 
  float                 thresh)
{
  float error           = fabs(ref - val) / val;
  float result          = 1.0f - error / thresh;
  return                (result < 0.0f) ? 0.0f : result;
}


#ifdef __cplusplus
}
#endif

#endif

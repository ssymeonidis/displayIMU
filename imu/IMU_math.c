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


// internal functions definitions
static inline float* norm4(float *v);
static inline float* scale(float *v, float m);  
static inline float* decrm(float *v, float *d);


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
* update quaternion with newest accelerometer datum
* assumes normalized quaternion and datum
******************************************************************************/

int IMU_math_estmAccl(
  float                 *q, 
  float                 *a, 
  float                 alpha, 
  float                 *FOM)
{
  // compute the objective function 
  float two_q[4]        = {2.0f*q[0], 2.0f*q[1], 2.0f*q[2], 2.0f*q[3]};
  float f_1             = two_q[1]*q[3] - two_q[0]*q[2] - a[0];
  float f_2             = two_q[0]*q[1] + two_q[2]*q[3] - a[1];
  float f_3             = 1.0f - two_q[1]*q[1] - two_q[2]*q[2] - a[2];
 
  // compute the Jacobian
  float J_11            = two_q[2]; 
  float J_12            = two_q[3];
  float J_13            = two_q[0];
  float J_14            = two_q[1];
  float J_32            = 2.0f * J_14;
  float J_33            = 2.0f * J_11;

  // calculate the gradient
  float qHatDot[4]      = {J_14 * f_2 - J_11 * f_1,
                           J_12 * f_1 + J_13 * f_2 - J_32 * f_3,
                           J_12 * f_2 - J_33 * f_3 - J_13 * f_1,
                           J_14 * f_1 + J_11 * f_2};
  norm4(decrm(q, scale(norm4(qHatDot), alpha)));
  *FOM                  = qHatDot[0];
  
  // exit (no errors)
  return 0;
}


/******************************************************************************
* update quaternion with newest accelerometer datum
* assumes normalized quaternion and datum
******************************************************************************/

int IMU_math_estmMagn(
  float                 *q, 
  float                 *m, 
  float                 alpha, 
  float                 *FOM)
{
  // compute the objective function 
  float twox            = 2.0f * m[2];
  float twoz            = 2.0f * m[0];
  float q1_q3           = q[0] * q[2];
  float q2_q4           = q[1] * q[3];
  float q3_q3           = q[2] * q[2];
  float f_4             = twox * (0.5f - q3_q3 - q[3]*q[3]) +
                          twoz * (q2_q4 - q1_q3) - m[2];
  float f_5             = twox * (q[1]*q[2] - q[0]*q[3]) +
                          twoz * (q[0]*q[1] + q[2]*q[3]) - m[1];
  float f_6             = twox * (q1_q3 + q2_q4) +
                          twoz * (0.5f - q[1]*q[1] - q3_q3) - m[0];

  // compute the Jacobian
  float twox_q[4]       = {twox * q[0], twox * q[1],
                           twox * q[2], twox * q[3]};
  float twoz_q[4]       = {twoz * q[0], twoz * q[1],
                           twoz * q[2], twoz * q[3]};
  float J_41            = twoz_q[2];
  float J_42            = twoz_q[3];
  float J_43            = 2.0f * twox_q[2] + twoz_q[0]; 
  float J_44            = 2.0f * twox_q[3] - twoz_q[1];
  float J_51            = twox_q[3] - twoz_q[1];
  float J_52            = twox_q[2] + twoz_q[0];
  float J_53            = twox_q[1] + twoz_q[3];
  float J_54            = twox_q[0] - twoz_q[2];
  float J_61            = twox_q[2];
  float J_62            = twox_q[3] - 2.0f * twoz_q[1];
  float J_63            = twox_q[0] - 2.0f * twoz_q[2];
  float J_64            = twox_q[1];

  // calculate the gradient
  float qHatDot[4]      = {-J_41 * f_4 - J_51 * f_5 + J_61 * f_6,
                            J_42 * f_4 + J_52 * f_5 + J_62 * f_6,
                           -J_43 * f_4 + J_53 * f_5 + J_63 * f_6,
                           -J_44 * f_4 - J_54 * f_5 + J_64 * f_6};
  norm4(decrm(q, scale(norm4(qHatDot), alpha)));
  *FOM                  = qHatDot[0];
  
  // exit (no errors)
  return 0;
}


/******************************************************************************
* update quaternion with newest accelerometer datum
* assumes normalized quaternion and datum
******************************************************************************/

float IMU_math_calcWeight(
  float                 val, 
  float                 ref, 
  float                 thresh)
{
  float error           = fabs(ref - val) / val;
  float result          = 1.0f - error / thresh;
  return                (result < 0.0f) ? 0.0f : result;
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


/******************************************************************************
* utility function - normalize 4x1 array
******************************************************************************/

inline float* norm4(
  float          *v)
{
  float norm     = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
  if (norm > 0.001f) {
    v[0]        /= norm;
    v[1]        /= norm;
    v[2]        /= norm;
    v[3]        /= norm;
  }
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

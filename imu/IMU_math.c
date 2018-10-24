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
* quaternion multiplication 
******************************************************************************/

float* IMU_math_quatMult(
  float                 *in1, 
  float                 *in2,
  float                 *out)
{
  out[0] = in2[0]*in1[0] - in2[1]*in1[1] - in2[2]*in1[2] - in2[3]*in1[3];
  out[1] = in2[0]*in1[1] + in2[1]*in1[0] - in2[2]*in1[3] + in2[3]*in1[2];
  out[2] = in2[0]*in1[2] + in2[1]*in1[3] + in2[2]*in1[0] - in2[3]*in1[1];
  out[3] = in2[0]*in1[3] - in2[1]*in1[2] + in2[2]*in1[1] + in2[3]*in1[0];
  return out;  // allows function to be used as function argument
}


/******************************************************************************
* quaternion multiplication w/ conjugagte
******************************************************************************/

float* IMU_math_quatMultConj(
  float                 *in1, 
  float                 *in2,
  float                 *out)
{
  out[0] = in2[0]*in1[0] + in2[1]*in1[1] + in2[2]*in1[2] + in2[3]*in1[3];
  out[1] = in2[0]*in1[1] - in2[1]*in1[0] + in2[2]*in1[3] - in2[3]*in1[2];
  out[2] = in2[0]*in1[2] - in2[1]*in1[3] - in2[2]*in1[0] + in2[3]*in1[1];
  out[3] = in2[0]*in1[3] + in2[1]*in1[2] - in2[2]*in1[1] - in2[3]*in1[0];
  return out;  // allows function to be used as function argument
}


/******************************************************************************
* rotate vector by quaternion (forward)
******************************************************************************/

float* IMU_math_rotateForward(
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

float* IMU_math_rotateReverse(
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
* convert quaternion to Euler angles
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
******************************************************************************/

float* IMU_math_quatToEuler(
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

float* IMU_math_eulerToQuat(
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

float* IMU_math_radToDeg(
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

float* IMU_math_degToRad(
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
* get up component from quaternion
******************************************************************************/

float* IMU_math_quatToUp(
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

float* IMU_math_quatToFrwd(
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

float* IMU_math_upToQuat(
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

float* IMU_math_upFrwdToQuat(
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

float* IMU_math_vectToQuat(
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

int IMU_math_estmMagnRef(
  float                 *q, 
  float                 *m, 
  float                 refx,
  float                 refz,
  float                 alpha, 
  float                 *FOM)
{
  // compute the objective function 
  float twox            = 2.0f * refx;
  float twoz            = 2.0f * refz;
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
* utility function - normalize 4x1 array
******************************************************************************/

inline float* norm4(
  float                 *v)
{
  float norm            = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2] + v[3]*v[3]);
  if (norm > 0.001f) {
    v[0]               /= norm;
    v[1]               /= norm;
    v[2]               /= norm;
    v[3]               /= norm;
  }
  return v;
}


/******************************************************************************
* utility function - scale 4x1 array by scalar value
******************************************************************************/

inline float* scale(
  float                 *v, 
  float                 m)
{
  v[0]                  *= m;
  v[1]                  *= m;
  v[2]                  *= m;
  v[3]                  *= m;
  return v;
}


/******************************************************************************
* utility function - decrement 4x1 array by another 4x1 array
******************************************************************************/

inline float* decrm(
  float                 *v, 
  float                 *d)
{
  v[0]                  -= d[0];
  v[1]                  -= d[1];
  v[2]                  -= d[2];
  v[3]                  -= d[3];
  return v;
}

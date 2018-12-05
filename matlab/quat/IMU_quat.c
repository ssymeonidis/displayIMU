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

#ifndef _IMU_QUAT_H
#define _IMU_QUAT_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements 
#include <math.h>            // sqrt/trig

// quaternion basic "math" operators
float  IMU_quat_mag           (float *q);
float* IMU_quat_norm          (float *q,  float *out);
float* IMU_quat_conj          (float *q,  float *out);
float* IMU_quat_mult          (float *q1, float *q2, float *out);
float* IMU_quat_multConj      (float *q1, float *q2, float *out);
float* IMU_quat_conjMult      (float *q1, float *q2, float *out);

// quaternion vector rotation operators
float* IMU_quat_rotateFrwd    (float *v,  float *q,  float *out);
float* IMU_quat_rotateRvrs    (float *v,  float *q,  float *out);
float* IMU_quat_toFrwd        (float *q,  float *v);
float* IMU_quat_toUp          (float *q,  float *v);
float* IMU_quat_toRght        (float *q,  float *v);

// converting between quaternions and pointing vectors
float* IMU_quat_fromFrwdSafe  (float *f,  float *q);
float* IMU_quat_fromFrwdNorm  (float *f,  float *q);
float* IMU_quat_fromFrwdFast  (float *f,  float *q);
float* IMU_quat_fromUpFast    (float *u,  float *q);
float* IMU_quat_fromUpSafe    (float *u,  float *q);
float* IMU_quat_fromFrwdUp    (float *f,  float *u,  float *q);
float* IMU_quat_fromUpFrwd    (float *u,  float *f,  float *q);
float* IMU_quat_fromVec       (float *u,  float *v,  float *q);

// converting between quaternions and Euler angles
float* IMU_quat_toRad         (float *q,  float *E);
float* IMU_quat_toDeg         (float *q,  float *E);
float* IMU_quat_fromRad       (float *E,  float *q);
float* IMU_quat_fromDeg       (float *E,  float *q);

// converting between quaternions and rotation matricies
float* IMU_quat_fromMatrix    (float *M,  float *q);
float* IMU_quat_toMatrix      (float *q,  float *M);


/******************************************************************************
* quaternion magnitude
******************************************************************************/

float  IMU_quat_mag(
  float       *q)
{
  return sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]); 
}


/******************************************************************************
* quaternion magnitude
******************************************************************************/

float* IMU_quat_norm(
  float       *q,
  float       *out)
{
  float mag   = IMU_quat_mag(q);
  out[0]      = q[0] / mag;
  out[1]      = q[1] / mag;
  out[2]      = q[2] / mag;
  out[3]      = q[3] / mag;
  return out; // allows function to be used as function argument
}


/******************************************************************************
* quaternion conjugate
******************************************************************************/

float* IMU_quat_conj(
  float       *q, 
  float       *out)
{
  out[0]      =  q[0];
  out[1]      = -q[1];
  out[2]      = -q[2];
  out[3]      = -q[3];
  return out; // allows function to be used as function argument
}


/******************************************************************************
* quaternion multiplication 
******************************************************************************/

float* IMU_quat_mult(
  float       *q1, 
  float       *q2,
  float       *out)
{
  out[0]      = q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2] - q2[3]*q1[3];
  out[1]      = q2[0]*q1[1] + q2[1]*q1[0] - q2[2]*q1[3] + q2[3]*q1[2];
  out[2]      = q2[0]*q1[2] + q2[1]*q1[3] + q2[2]*q1[0] - q2[3]*q1[1];
  out[3]      = q2[0]*q1[3] - q2[1]*q1[2] + q2[2]*q1[1] + q2[3]*q1[0];
  return out; // allows function to be used as function argument
}


/******************************************************************************
* quaternion multiplication w/ conjugagte (q * q')
******************************************************************************/

float* IMU_quat_multConj(
  float       *q1, 
  float       *q2,
  float       *out)
{
  out[0]      = q2[0]*q1[0] + q2[1]*q1[1] + q2[2]*q1[2] + q2[3]*q1[3];
  out[1]      = q2[0]*q1[1] - q2[1]*q1[0] + q2[2]*q1[3] - q2[3]*q1[2];
  out[2]      = q2[0]*q1[2] - q2[1]*q1[3] - q2[2]*q1[0] + q2[3]*q1[1];
  out[3]      = q2[0]*q1[3] + q2[1]*q1[2] - q2[2]*q1[1] - q2[3]*q1[0];
  return out; // allows function to be used as function argument
}


/******************************************************************************
* quaternion multiplication w/ conjugagte (q' * q)
******************************************************************************/

float* IMU_quat_conjMult(
  float       *q1, 
  float       *q2,
  float       *out)
{
  out[0]      = q2[0]*q1[0] + q2[1]*q1[1] + q2[2]*q1[2] + q2[3]*q1[3];
  out[1]      = q2[1]*q1[0] - q2[0]*q1[1] + q2[2]*q1[3] - q2[3]*q1[2];
  out[2]      = q2[2]*q1[0] - q2[0]*q1[2] - q2[1]*q1[3] + q2[3]*q1[1];
  out[3]      = q2[3]*q1[0] - q2[0]*q1[3] + q2[1]*q1[2] - q2[2]*q1[1];
  return out; // allows function to be used as function argument
}


/******************************************************************************
* rotate vector by quaternion (forward)
******************************************************************************/

float* IMU_quat_rotateFrwd(
  float       *v, 
  float       *q,
  float       *out)
{
  out[0]      = 2.0f * (v[0] * (0.5f - q[2]*q[2] - q[3]*q[3])
                      + v[1] * (q[1]*q[2] - q[0]*q[3]) 
                      + v[2] * (q[1]*q[3] + q[0]*q[2]));
  out[1]      = 2.0f * (v[0] * (q[1]*q[2] + q[0]*q[3])
                      + v[1] * (0.5f - q[1]*q[1] - q[3]*q[3])
                      + v[2] * (q[2]*q[3] - q[0]*q[1]));
  out[2]      = 2.0f * (v[0] * (q[1]*q[3] - q[0]*q[2])
                      + v[1] * (q[2]*q[3] + q[0]*q[1])
                      + v[2] * (0.5f - q[1]*q[1] - q[2]*q[2]));
  return out;
}


/******************************************************************************
* rotate vector by quaternion (reverse)
******************************************************************************/

float* IMU_quat_rotateRvrs(
  float       *v, 
  float       *q,
  float       *out)
{
  out[0]      = 2.0f * (v[0] * (0.5f - q[2]*q[2] - q[3]*q[3])
                      + v[1] * (q[1]*q[2] + q[0]*q[3]) 
                      + v[2] * (q[1]*q[3] - q[0]*q[2]));
  out[1]      = 2.0f * (v[0] * (q[1]*q[2] - q[0]*q[3])
                      + v[1] * (0.5f - q[1]*q[1] - q[3]*q[3])
                      + v[2] * (q[2]*q[3] + q[0]*q[1]));
  out[2]      = 2.0f * (v[0] * (q[1]*q[3] + q[0]*q[2])
                      + v[1] * (q[2]*q[3] - q[0]*q[1])
                      + v[2] * (0.5f - q[1]*q[1] - q[2]*q[2]));
  return out;
}


/******************************************************************************
* get forward component from quaternion
******************************************************************************/

float* IMU_quat_toFrwd(
  float       *q, 
  float       *v)
{
  v[0]        =  2.0f * (0.5f - q[2]*q[2] - q[3]*q[3]);
  v[1]        =  2.0f * (q[1]*q[2] + q[0]*q[3]);
  v[2]        =  2.0f * (q[1]*q[3] - q[0]*q[2]);
  return v;
}


/******************************************************************************
* get up component from quaternion
******************************************************************************/

float* IMU_quat_toUp(
  float       *q,
  float       *v)
{
  v[0]        = -2.0f * (q[1]*q[3] + q[0]*q[2]);
  v[1]        = -2.0f * (q[2]*q[3] - q[0]*q[1]);
  v[2]        = -2.0f * (0.5f - q[1]*q[1] - q[2]*q[2]);
  return v;
}


/******************************************************************************
* get right component from quaternion
******************************************************************************/

float* IMU_quat_toRght(
  float       *q, 
  float       *v)
{
  v[0]        =  2.0f * (q[1]*q[2] - q[0]*q[3]);
  v[1]        =  2.0f * (0.5f - q[1]*q[1] - q[3]*q[3]);
  v[2]        =  2.0f * (q[2]*q[3] + q[0]*q[1]);
  return v;
}


/******************************************************************************
* set quaternion from forward vector (constrains roll)
******************************************************************************/

float* IMU_quat_fromFrwdSafe(
  float       *v,
  float       *q)
{
  float norm  = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  float E[3]  = {atan2f(v[1], v[0]), asin(-v[2] / norm), 0.0f};
  return IMU_quat_fromRad(E, q);
}


/******************************************************************************
* set quaternion from forward vector (ignores pitch and roll)
******************************************************************************/

float* IMU_quat_fromFrwdNorm(
  float       *v,
  float       *q)
{
  float ang   = 0.5f * atan2f(v[1], v[0]);
  q[0]        = cosf(ang);
  q[1]        = 0.0f;
  q[2]        = 0.0f;
  q[3]        = sinf(ang);
  return q;
}


/******************************************************************************
* set quaternion from forward vector (optimized for speed)
******************************************************************************/

float* IMU_quat_fromFrwdFast(
  float       *v,
  float       *q)
{
  float norm  = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  q[0]        = norm + v[0];
  if (q[0] > 0.001f * norm) {
    norm      = sqrtf(q[0]*q[0] + v[1]*v[1] + v[2]*v[2]);
    q[0]      =  q[0] / norm;
    q[1]      =  0.0f;
    q[2]      = -v[2] / norm;
    q[3]      =  v[1] / norm;
  } else {
    q[0]      =  1.0f;
    q[1]      =  0.0f;
    q[2]      =  0.0f;
    q[3]      =  0.0f;
  }
  return q;
}


/******************************************************************************
* set quaternion from up vector (constrains azimuth)
******************************************************************************/

float* IMU_quat_fromUpSafe(
  float       *v,
  float       *q)
{
  float norm  = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  float E[3]  = {0.0f, atan2f(-v[0], -v[2]), asin(v[1] / norm)};
  return IMU_quat_fromRad(E, q);
}


/******************************************************************************
* set quaternion from up vector (optimized for speed)
******************************************************************************/

float* IMU_quat_fromUpFast(
  float       *v,
  float       *q)
{
  float norm  = sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  q[0]        = norm - v[2];
  if (q[0] > 0.001f * norm) {
    norm      = sqrtf(q[0]*q[0] + v[0]*v[0] + v[1]*v[1]);
    q[0]      =  q[0] / norm;
    q[1]      =  v[1] / norm;
    q[2]      = -v[0] / norm;
    q[3]      =  0.0f;
  } else {
    q[0]      =  1.0f;
    q[1]      =  0.0f;
    q[2]      =  0.0f;
    q[3]      =  0.0f;
  }
  return q;
}


/******************************************************************************
* get quaternion from forward and up vectors (ortho-normalize up)
******************************************************************************/

float* IMU_quat_fromFrwdUp(
  float       *f,
  float       *u,
  float       *q)
{
  // define internal variables
  float       M[9];
  float       n;

  // normalize forward vector
  n           = sqrtf(f[0]*f[0] + f[1]*f[1] + f[2]*f[2]);
  M[0]        = f[0]/n;
  M[1]        = f[1]/n;
  M[2]        = f[2]/n;

  // ortho-normalize up vector 
  n           = M[0]*u[0] + M[1]*u[1] + M[2]*u[2];
  M[6]        = u[0] - n*M[0];
  M[7]        = u[1] - n*M[1];
  M[8]        = u[2] - n*M[2];
  n           = sqrtf(M[6]*M[6] + M[7]*M[7] + M[8]*M[8]);
  M[6]        = -M[6] / n;
  M[7]        = -M[7] / n;
  M[8]        = -M[8] / n;

  // calcuate the right vector (cross product)
  M[3]        = M[7]*M[2] - M[8]*M[1];
  M[4]        = M[8]*M[0] - M[6]*M[2];
  M[5]        = M[6]*M[1] - M[7]*M[0];

  // calculate the quaternion
  return IMU_quat_fromMatrix(M, q);
}


/******************************************************************************
* get quaternion from forward and up vectors (ortho-normalize forward)
******************************************************************************/

float* IMU_quat_fromUpFrwd(
  float       *u,
  float       *f,
  float       *q)
{
  // define internal variables
  float       M[9];
  float       n;

  // normalize up vector
  n           = sqrtf(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
  M[6]        = -u[0]/n;
  M[7]        = -u[1]/n;
  M[8]        = -u[2]/n;

  // ortho-normalize forward vector 
  n           = M[6]*f[0] + M[7]*f[1] + M[8]*f[2];
  M[0]        = f[0] - n*M[6];
  M[1]        = f[1] - n*M[7];
  M[2]        = f[2] - n*M[8];
  n           = sqrtf(M[0]*M[0] + M[1]*M[1] + M[2]*M[2]);
  M[0]        = M[0] / n;
  M[1]        = M[1] / n;
  M[2]        = M[2] / n;

  // calcuate the right vector (cross product)
  M[3]        = M[7]*M[2] - M[8]*M[1];
  M[4]        = M[8]*M[0] - M[6]*M[2];
  M[5]        = M[6]*M[1] - M[7]*M[0];

  // calculate the quaternion
  return IMU_quat_fromMatrix(M, q);
}


/******************************************************************************
* get forward component from quaternion
******************************************************************************/

float* IMU_quat_fromVec(
  float       *u, 
  float       *v,
  float       *q)
{
  float norm  = sqrtf((u[0]*u[0] + u[1]*u[1] + u[2]*u[2]) *
                      (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]));
  q[0]        = norm + u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
  if (q[0] > 0.001f * norm) {
    q[1]      = u[1]*v[2] - u[2]*v[1];
    q[2]      = u[2]*v[0] - u[0]*v[2];
    q[3]      = u[0]*v[1] - u[1]*v[0];
  } else if (fabs(u[0]) > fabs(u[2])) {
    q[1]      = -u[1];
    q[2]      =  u[0];
    q[3]      =  0.0f;
  } else {
    q[1]      =  0.0f;
    q[2]      = -u[2];
    q[3]      =  u[1];
  }
  norm        = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  q[0]        = q[0] / norm;
  q[1]        = q[1] / norm;
  q[2]        = q[2] / norm;
  q[3]        = q[3] / norm;
  return q;
}


/******************************************************************************
* convert quaternion to Euler angles (radians)
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
******************************************************************************/

float* IMU_quat_toRad(
  float       *q, 
  float       *E)
{
  // roll (x-axis rotation)
  float sinr  = 2.0f * (q[0]*q[1] + q[2]*q[3]);
  float cosr  = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
  E[2]        = atan2(sinr, cosr);

  // pitch (y-axis rotation)
  float sinp  = 2.0f * (q[0]*q[2] - q[3]*q[1]);
  if (fabs(sinp) >= 1)
    E[1]      = copysign(M_PI/2, sinp);
  else
    E[1]      = asin(sinp);

  // yaw (z-axis rotation)
  float siny  = 2.0f * (q[0]*q[3] + q[1]*q[2]);
  float cosy  = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
  E[0]        = atan2(siny, cosy);
  
  // pass Euler angle
  return E; 
}


/******************************************************************************
* convert quaternion to Euler angles (degrees)
******************************************************************************/

float* IMU_quat_toDeg(
  float       *q, 
  float       *E)
{
  float scale = 180.0f / M_PI;
  IMU_quat_toRad(q, E);
  E[0]        = scale * E[0];
  E[1]        = scale * E[1];
  E[2]        = scale * E[2];
  return E;
}


/******************************************************************************
* convert Euler angles to quaternion (radians)
* https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
******************************************************************************/

float* IMU_quat_fromRad(
  float       *E, 
  float       *q)
{
  // angle function abreviations
  float cy    = cos(E[0] * 0.5f);
  float sy    = sin(E[0] * 0.5f);
  float cr    = cos(E[2] * 0.5f);
  float sr    = sin(E[2] * 0.5f);
  float cp    = cos(E[1] * 0.5f);
  float sp    = sin(E[1] * 0.5f);

  // conversion to quaternion
  q[0]        = cy*cr*cp + sy*sr*sp;
  q[1]        = cy*sr*cp - sy*cr*sp;
  q[2]        = cy*cr*sp + sy*sr*cp;
  q[3]        = sy*cr*cp - cy*sr*sp;
  return q;
}


/******************************************************************************
* convert radians to degrees
******************************************************************************/

float* IMU_quat_fromDeg(
  float       *E, 
  float       *q)
{
  float scale = M_PI / 180.0f;
  float r[3]  = {scale * E[0], scale * E[1], scale * E[2]};
  IMU_quat_fromRad(r, q);
  return q;
}


/******************************************************************************
* convert radians to degrees
******************************************************************************/

float* IMU_quat_fromMatrix(
  float       *M, 
  float       *q)
{
  float n     = M[0]+M[4]+M[8];
  if (n > 0) {
    n         = sqrtf(1.0+n)*2.0f;
    q[0]      = 0.25f*n;
    q[1]      = (M[5]-M[7])/n;
    q[2]      = (M[6]-M[2])/n;
    q[3]      = (M[1]-M[3])/n;
  } else if (M[0] > M[4] && M[0] > M[8]) {
    n         = sqrtf(1.0+M[0]-M[4]-M[8])*2.0f;
    q[0]      = (M[5]-M[7])/n;
    q[1]      = 0.25f*n;
    q[2]      = (M[3]+M[1])/n;
    q[3]      = (M[6]+M[2])/n;
  } else if (M[4] > M[8]) {
    n         = sqrtf(1.0+M[4]-M[0]-M[8])*2.0f;
    q[0]      = (M[6]-M[2])/n;
    q[1]      = (M[3]+M[1])/n;
    q[2]      = 0.25f*n;
    q[3]      = (M[7]+M[5])/n;
  } else {
    n         = sqrtf(1.0+M[8]-M[0]-M[4])*2.0f;
    q[0]      = (M[1]-M[3])/n;
    q[1]      = (M[6]+M[2])/n;
    q[2]      = (M[7]+M[5])/n;
    q[3]      = 0.25f*n;
  }
  return M;
}


/******************************************************************************
* convert radians to degrees
******************************************************************************/

float* IMU_quat_toMatrix(
  float       *q, 
  float       *M)
{
  M[0]        = 1.0f - 2.0f * (q[2]*q[2] + q[3]*q[3]);
  M[1]        = 2.0f * (q[1]*q[2] + q[0]*q[3]);
  M[2]        = 2.0f * (q[1]*q[3] - q[0]*q[2]);
  M[3]        = 2.0f * (q[1]*q[2] - q[0]*q[3]);
  M[4]        = 1.0f - 2.0f * (q[1]*q[1] + q[3]*q[3]);
  M[5]        = 2.0f * (q[2]*q[3] + q[0]*q[1]);
  M[6]        = 2.0f * (q[1]*q[3] + q[0]*q[2]);
  M[7]        = 2.0f * (q[2]*q[3] - q[0]*q[1]);
  M[8]        = 1.0f - 2.0f * (q[1]*q[1] + q[2]*q[2]);
  return M;
}


#ifdef __cplusplus
}
#endif

#endif

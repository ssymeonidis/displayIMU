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
 
  // calculate the gradient
  float qHatDot[4]      = {two_q[1]*f_2 - two_q[2]*f_1,
                           two_q[3]*f_1 + two_q[0]*f_2 - 2*two_q[1]*f_3,
                           two_q[3]*f_2 - 2*two_q[2]*f_3 - two_q[0]*f_1,
                           two_q[1]*f_1 + two_q[2]*f_2};
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

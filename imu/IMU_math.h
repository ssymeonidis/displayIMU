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

// core filters
static inline float IMU_math_calcWeight (float val, float ref, float thresh);
static inline int   IMU_math_estmGyro   (float *q,  float *g,  float dt);
int    IMU_math_estmAccl      (float *q, float *a, float alpha, float *FOM);
int    IMU_math_estmMagnNorm  (float *q, float *m, float alpha, float *FOM);
int    IMU_math_estmMagnRef   (float *q, float *m, float refx,  float refz,
                               float alpha, float *FOM);


/******************************************************************************
* function used to apply target and threshold to generate weight 
******************************************************************************/

inline float IMU_math_calcWeight(
  float                 val, 
  float                 ref, 
  float                 thresh)
{
  if (thresh < 0.01)
    return              1.0f;
  float error           = fabs(ref - val) / thresh;
  float result          = 1.0f - error;
  return                (result < 0.0f) ? 0.0f : result;
}


/******************************************************************************
* function used to apply gyroscope rates 
******************************************************************************/

inline int   IMU_math_estmGyro(
  float                 *q,  
  float                 *g,  
  float                 dt)
{
  float half_dt         = 0.5 * dt;
  float dq[4]           = {-q[1]*g[0] - q[2]*g[1] - q[3]*g[2],
                            q[0]*g[0] + q[2]*g[2] - q[3]*g[1],
                            q[0]*g[1] - q[1]*g[2] + q[3]*g[0],
                            q[0]*g[2] + q[1]*g[1] - q[2]*g[0]}; 
  q[0]                 += half_dt * dq[0];
  q[1]                 += half_dt * dq[1];
  q[2]                 += half_dt * dq[2];
  q[3]                 += half_dt * dq[3];
  return 0;
}


#ifdef __cplusplus
}
#endif

#endif

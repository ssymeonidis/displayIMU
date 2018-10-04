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
* set reference using last system quaternion
******************************************************************************/

void IMU_math_calcRefState(
  float                 *q, 
  float                 *ref)
{
  ref[0]       =  q[0];
  ref[1]       = -q[1];
  ref[2]       = -q[2];
  ref[3]       = -q[3];
}


/******************************************************************************
* set reference using up vector
* assumes: normalized input
******************************************************************************/

void IMU_math_calcRefVectUp(
  float                 *u, 
  float                 *ref)
{
  // updating reference based on accelerometer
  float  tmp         = sqrt(2.0 + 2.0 * u[2]);
  if (tmp > 0.001) {
    ref[0]      =  0.5  * tmp;
    ref[1]      = -u[1] / tmp;
    ref[2]      =  u[0] / tmp;
    ref[3]      =  0.0;
  } else {
    ref[0]      =  0.0;
    ref[1]      =  0.0;
    ref[2]      =  1.0;
    ref[3]      =  0.0;
  }
}


/******************************************************************************
* apply reference quaterion to current state
******************************************************************************/

void IMU_math_applyRef(
  float                 *q, 
  float                 *ref, 
  float                 *q_out)
{
  q_out[0] = q[0]*ref[0] - q[1]*ref[1] - q[2]*ref[2] - q[3]*ref[3];
  q_out[1] = q[0]*ref[1] + q[1]*ref[0] + q[2]*ref[3] - q[3]*ref[2];
  q_out[2] = q[0]*ref[2] - q[1]*ref[3] + q[2]*ref[0] + q[3]*ref[1];
  q_out[3] = q[0]*ref[3] + q[1]*ref[2] - q[2]*ref[1] + q[3]*ref[0];
}


/******************************************************************************
* calculate euler angle
******************************************************************************/

void IMU_util_calcEuler(
  float                 *q, 
  float                 *E)
{
  float Q[4] = {q[0]*q[0], q[1]*q[1], q[2]*q[2], q[3]*q[3]};
  E[0] = 180 * atan2(2*(q[1]*q[3]+q[3]*q[0]),  Q[1]-Q[2]-Q[3]+Q[0]) / M_PI;
  E[1] = 180 * asin(-2*(q[1]*q[3]-q[2]*q[0])) / M_PI;
  E[2] = 180 * atan2(2*(q[2]*q[3]+q[1]*q[0]), -Q[1]-Q[2]+Q[3]+Q[0]) / M_PI;
}

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

// include statements 
#include "IMU_correct.h"

// internally managed structures
struct IMU_correct_config   config[IMU_MAX_INST];
static unsigned short       IMU_correct_inst = 0;


/******************************************************************************
* function to return config structure handle
******************************************************************************/

int IMU_correct_init(unsigned short *id, struct IMU_correct_config **pntr) 
{
  // check for device count overflow
  if (IMU_correct_inst >= IMU_MAX_INST)
    return IMU_CORRECT_INST_OVERFLOW;

  // return handle and config pointer
  *id   = IMU_correct_inst; 
  *pntr = &config[*id];
  IMU_correct_inst++;
  return 0;
}


/******************************************************************************
* correct raw gyroscope data
******************************************************************************/

void IMU_correct_gyro(unsigned short id, IMU_TYPE *g_raw, IMU_TYPE *g)
{
  // define internal variables
  float *bias  = config[id].gBias;
  float *mult  = config[id].gMult; 

  // apply bias
  g[0]         = g_raw[0] + bias[0];
  g[1]         = g_raw[1] + bias[1];
  g[2]         = g_raw[2] + bias[2];

  // apply transform
  g[0]         = g[0]*mult[0] + g[1]*mult[1] + g[2]*mult[2];
  g[1]         = g[0]*mult[3] + g[1]*mult[4] + g[2]*mult[5];
  g[2]         = g[0]*mult[6] + g[1]*mult[7] + g[2]*mult[8];
}


/******************************************************************************
* correct raw accelerometer data
******************************************************************************/

void IMU_correct_accl(unsigned short id, IMU_TYPE *a_raw, IMU_TYPE *a)
{
  // define internal variables
  float *bias  = config[id].aBias;
  float *mult  = config[id].aMult; 

  // apply bias
  a[0]         = a_raw[0] - bias[0];
  a[1]         = a_raw[1] - bias[1];
  a[2]         = a_raw[2] - bias[2];

  // apply transform
  a[0]         = a[0]*mult[0] + a[1]*mult[1] + a[2]*mult[2];
  a[1]         = a[0]*mult[3] + a[1]*mult[4] + a[2]*mult[5];
  a[2]         = a[0]*mult[6] + a[1]*mult[7] + a[2]*mult[8];
}


/******************************************************************************
* correct raw magnetometer data
******************************************************************************/

void IMU_correct_magn(unsigned short id, IMU_TYPE *m_raw, IMU_TYPE *m)

{
  // define internal variables
  float *bias  = config[id].mBias;
  float *mult  = config[id].mMult; 

  // apply bias
  m[0]         = m_raw[0] - bias[0];
  m[1]         = m_raw[1] - bias[1];
  m[2]         = m_raw[2] - bias[2];

  // apply transform
  m[0]         = m[0]*mult[0] + m[1]*mult[1] + m[2]*mult[2];
  m[1]         = m[0]*mult[3] + m[1]*mult[4] + m[2]*mult[5];
  m[2]         = m[0]*mult[6] + m[1]*mult[7] + m[2]*mult[8];
}


/******************************************************************************
* correct raw gyroscope, accelerometer, and magnetometer data
******************************************************************************/

void IMU_correct_all(unsigned short id, 
                     IMU_TYPE *g_raw, IMU_TYPE *a_raw, IMU_TYPE *m_raw,
                     IMU_TYPE *g,     IMU_TYPE *a,     IMU_TYPE *m)
{
  IMU_correct_gyro(id, g_raw, g);
  IMU_correct_accl(id, a_raw, a);
  IMU_correct_magn(id, m_raw, m);
}

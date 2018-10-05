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
#include "IMU_rect.h"

// internally managed structures
IMU_rect_config  config[IMU_MAX_INST];
uint16_t         numInstRect = 0;


/******************************************************************************
* function to return config structure handle
******************************************************************************/

int IMU_rect_init(
  uint16_t              *id, 
  IMU_rect_config       **pntr)
{
  // check for device count overflow
  if (numInstRect >= IMU_MAX_INST)
    return IMU_RECT_INST_OVERFLOW;

  // return handle and config pointer
  *id    = numInstRect; 
  *pntr  = &config[*id];
  numInstRect++;
  return 0;
}


/******************************************************************************
* function to copy config structure
******************************************************************************/

int IMU_rect_getConfig(
  uint16_t              id, 
  IMU_rect_config       **pntr)
{
  // check for out-of-bounds condition
  if (id > numInstRect-1)
    return IMU_RECT_BAD_INST; 

  // return state
  *pntr = &config[id];
  return 0;
}


/******************************************************************************
* correct raw gyroscope data
******************************************************************************/

int IMU_rect_gyro(
  uint16_t              id, 
  IMU_TYPE              *g_raw, 
  IMU_TYPE              *g_out)
{
  // check execution condition
  if (id > numInstRect-1)
    return IMU_RECT_BAD_INST;
  if (!config[id].enable)
    return 0;

  // apply bias
  float *bias  = config[id].gBias;
  float g[3]   = {(float)g_raw[0] + bias[0], 
                  (float)g_raw[1] + bias[1],
                  (float)g_raw[2] + bias[2]};

  // apply transform
  float *mult  = config[id].gMult; 
  g_out[0]     = (IMU_TYPE)(g[0]*mult[0] + g[1]*mult[1] + g[2]*mult[2]);
  g_out[1]     = (IMU_TYPE)(g[0]*mult[3] + g[1]*mult[4] + g[2]*mult[5]);
  g_out[2]     = (IMU_TYPE)(g[0]*mult[6] + g[1]*mult[7] + g[2]*mult[8]);

  // exit function
  return 0;
}


/******************************************************************************
* correct raw accelerometer data
******************************************************************************/

int IMU_rect_accl(
  uint16_t              id, 
  IMU_TYPE              *a_raw, 
  IMU_TYPE              *a_out)
{
  // check execution condition
  if (id > numInstRect-1)
    return IMU_RECT_BAD_INST;
  if (!config[id].enable)
    return 0;

  // apply bias
  float *bias  = config[id].aBias;
  float a[3]   = {(float)a_raw[0] + bias[0],
                  (float)a_raw[1] + bias[1],
                  (float)a_raw[2] + bias[2]};

  // apply transform
  float *mult  = config[id].aMult; 
  a_out[0]     = (IMU_TYPE)(a[0]*mult[0] + a[1]*mult[1] + a[2]*mult[2]);
  a_out[1]     = (IMU_TYPE)(a[0]*mult[3] + a[1]*mult[4] + a[2]*mult[5]);
  a_out[2]     = (IMU_TYPE)(a[0]*mult[6] + a[1]*mult[7] + a[2]*mult[8]);

  // exit function
  return 0;
}


/******************************************************************************
* correct raw magnetometer data
******************************************************************************/

int IMU_rect_magn(
  uint16_t              id, 
  IMU_TYPE              *m_raw, 
  IMU_TYPE              *m_out)

{
  // check execution condition
  if (id > numInstRect-1)
    return IMU_RECT_BAD_INST;
  if (!config[id].enable)
    return 0;

  // apply bias
  float *bias  = config[id].mBias;
  float m[3]   = {(float)m_raw[0] - bias[0],
                  (float)m_raw[1] - bias[1],
                  (float)m_raw[2] - bias[2]};

  // apply transform
  float *mult  = config[id].mMult; 
  m_out[0]     = (IMU_TYPE)(m[0]*mult[0] + m[1]*mult[1] + m[2]*mult[2]);
  m_out[1]     = (IMU_TYPE)(m[0]*mult[3] + m[1]*mult[4] + m[2]*mult[5]);
  m_out[2]     = (IMU_TYPE)(m[0]*mult[6] + m[1]*mult[7] + m[2]*mult[8]);
  
  // exit function
  return 0;
}


/******************************************************************************
* correct raw gyroscope, accelerometer, and magnetometer data
******************************************************************************/

int IMU_rect_all(
  uint16_t              id, 
  IMU_TYPE              *g_raw, 
  IMU_TYPE              *a_raw, 
  IMU_TYPE              *m_raw,
  IMU_TYPE              *g,     
  IMU_TYPE              *a,     
  IMU_TYPE              *m)
{
  // check execution condition
  if (id > numInstRect-1)
    return IMU_RECT_BAD_INST;
  if (!config[id].enable)
    return 0;

  IMU_rect_gyro(id, g_raw, g);
  IMU_rect_accl(id, a_raw, a);
  IMU_rect_magn(id, m_raw, m);
  return 0;
}

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
// #include <math.h>           // sqrt/trig
#include "IMU_correct.h"

// internally managed structures
struct IMU_correct_calib    calib;
struct IMU_correct_autocal  autocal;


/******************************************************************************
* function to return calib structure handle
******************************************************************************/

void IMU_correct_getCalib(struct IMU_correct_calib **calib_pntr) 
{
  *calib_pntr = &calib;
}


/******************************************************************************
* function to return autocal structure handle
******************************************************************************/

void IMU_correct_getAutocal(struct IMU_correct_autocal **autocal_pntr) 
{
  *autocal_pntr = &autocal;
}


/******************************************************************************
* correct raw gyroscope data
******************************************************************************/

void IMU_correct_gyro(float* g_raw, float* g)
{
  g[0]      = (g_raw[0] - calib.gBias[0]) / calib.gMult[0];
  g[1]      = (g_raw[1] - calib.gBias[1]) / calib.gMult[1];
  g[2]      = (g_raw[2] - calib.gBias[2]) / calib.gMult[2];
}


/******************************************************************************
* correct raw accelerometer data
******************************************************************************/

void IMU_correct_accl(float* a_raw, float* a)
{
  a[0]      = a_raw[0] - calib.aBias[0];
  a[1]      = a_raw[1] - calib.aBias[1];
  a[2]      = a_raw[2] - calib.aBias[2];
}


/******************************************************************************
* correct raw magnetometer data
******************************************************************************/

void IMU_correct_magn(float* m_raw, float* m)
{
  m[0]      = m_raw[0] - calib.mBias[0];
  m[1]      = m_raw[1] - calib.mBias[1];
  m[2]      = m_raw[2] - calib.mBias[2];
}


/******************************************************************************
* correct raw gyroscope, accelerometer, and magnetometer data
******************************************************************************/

void IMU_correct_all(float* g_raw, float* a_raw, float* m_raw,
                     float* g,     float* a,     float* m)
{
  IMU_correct_gyro(g_raw, g);
  IMU_correct_accl(a_raw, a);
  IMU_correct_magn(m_raw, m);
}

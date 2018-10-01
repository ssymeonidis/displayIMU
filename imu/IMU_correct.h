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

#ifndef _IMU_CORRECT_H
#define _IMU_CORRECT_H

#ifdef __cplusplus
extern "C" {
#endif

// define error codes
#define IMU_CORRECT_INST_OVERFLOW -1

// define the calibration structure (values tuned for each unit)
struct IMU_correct_calib {
  float          gBias[3];        // gyroscope biases
  float          gMult[9];        // gyroscope transform matrix 
  float          aBias[3];        // accelerometer biases
  float          aMult[9];        // accelerometer transform matrix 
  float          mBias[3];        // magnetometer biases
  float          mMult[9];        // magnetometer transform matrix
};

// data structure access functions
int  IMU_correct_init (unsigned short *id, struct IMU_correct_calib **calib);

// raw data correction functions
void IMU_correct_gyro (unsigned short id, IMU_TYPE *g_raw, IMU_TYPE *g);
void IMU_correct_accl (unsigned short id, IMU_TYPE *a_raw, IMU_TYPE *a);
void IMU_correct_magn (unsigned short id, IMU_TYPE *m_raw, IMU_TYPE *m);
void IMU_correct_all  (unsigned short id, 
                       IMU_TYPE *g_raw, IMU_TYPE *a_raw, IMU_TYPE *m_raw, 
                       IMU_TYPE *g,     IMU_TYPE *a,     IMU_TYPE *m);

#ifdef __cplusplus
}
#endif

#endif

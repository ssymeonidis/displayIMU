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

// define the calibration structure (values tuned for each unit)
struct IMU_correct_calib {
  float          gBias[3];        // gyroscope biases
  float          gMult[9];        // gyroscope transform matrix 
  float          aBias[3];        // accelerometer biases
  float          aMult[9];        // accelerometer transform matrix 
  float          mBias[3];        // magnetometer biases
  float          mMult[9];        // magnetometer transform matrix
  float          aMag;            // gravity magnitude
  float          mMag;            // magnetic north magnitude  
  float          mAng;            // magnetic north angle
};

// define the auto_calib state (used to improve config struct)
struct IMU_correct_autocal {
  float          gBias[3];        // gyroscope autocal estimate #1
  float          gBiasCont[3];    // gyroscope autocal estimate #2
  float          aMag;            // gravity magnitude estimate #1
  float          aMagCont;        // gravity magnitude estimate #2
  float          mMag;            // magnetic north magnitude estimate #1
  float          mMagCont;        // magnetic north magnitude estimate #2
  float          mAng;            // magnetic north angle estimate #1
  float          mAngCont;        // magnetic north angle estiamte #2
}; 


// data structure access functions
void IMU_correct_getCalib    (struct IMU_correct_calib   **calib);
void IMU_correct_getAutocal  (struct IMU_correct_autocal **autocal);

// raw data correction functions
void IMU_correct_gyro        (float* g_raw, float* g);
void IMU_correct_accl        (float* a_raw, float* a);
void IMU_correct_magn        (float* m_raw, float* m);
void IMU_correct_all         (float* g_raw, float* a_raw, float* m_raw, 
                              float* g,     float* a,     float* m);

#ifdef __cplusplus
}
#endif

#endif

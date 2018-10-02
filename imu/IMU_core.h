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

#ifndef _IMU_CORE_H
#define _IMU_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

// define error codes
#define IMU_CORE_INST_OVERFLOW -1


// define the configuration structure (values tuned for a part)
struct IMU_core_config {
  unsigned char       isGyro;          // enable gyroscope data
  unsigned char       isAccl;          // enable accelerometer data
  unsigned char       isMagn;          // enable magnetometer data
  unsigned char       isStable;        // enable detection of no movement
  unsigned char       isFOM;           // enable FOM calculation
  unsigned char       isMove;          // enable acceleration estimate
  float               aWeight;         // accelerometer IMU weight
  float               aMag;            // gravity magnitude
  float               aMagThresh;      // gravity magnitude error thresh
  float               mWeight;         // magnetometer IMU weight
  float               mMag;            // magnetic north magn  
  float               mMagThresh;      // magnetic north magn error thres
  float               mAng;            // magnetic north angle
  float               mAngThresh;      // magnetic north angle error thresh
  float               gThreshVal;      // no motion threshold value
  float               gThreshTime;     // no motion threhsold time
  float               moveAlpha;       // acceleration estimate alpha
};

// define internal state struct (captures internal IMU state)
struct IMU_core_state {
  float               t;               // last datum time
  float               t_stable;        // last "unstable" time
  float               SEq[4];          // current quaterion
  float               A[3];            // last acceleration estimate
  float               A_rot[3];        // rotated acceleration estimate
  float               a[3];            // last acclerometer datum
  unsigned char       aReset;          // accelerometer reset signal
  unsigned char       mReset;          // magnetometer reset signal
};

// define datum enumertation type for FOM structure
enum IMU_core_sensor_type {
  all                         = 0,
  gyro                        = 1,
  accl                        = 2,
  magn                        = 3
};

// define gyroscope figure of merit
struct IMU_core_FOM_gyro {
  unsigned char               stable;
};

// define acclerometer figure of merit
struct IMU_core_FOM_accl {
  float                       aMag;
  float                       aDelt;
};
  
// define magnetometer figure of merit
struct IMU_core_FOM_magn {
  float                       mMag;
  float                       mAng;
  float                       mDelt;
};

// define figure of merit structure
struct IMU_core_FOM {
  enum IMU_core_sensor_type   type;
  union {
    struct IMU_core_FOM_gyro  gyro;
    struct IMU_core_FOM_accl  accl;
    struct IMU_core_FOM_magn  magn;
  }                           data;
};


// data structure access functions
int IMU_core_init(
  unsigned short              *id, 
  struct IMU_core_config      **config);
void IMU_core_getState(
  unsigned short              id,  
  struct IMU_core_state       **state);


// general operation functions 
void IMU_core_reset(
  unsigned short              id);
float* IMU_core_deadRecon(
  unsigned short              id,  
  float                       t, 
  float                       *a, 
  float                       *m);
float* IMU_core_estmGyro(
  unsigned short              id, 
  float                       t, 
  float                       *g, 
  struct IMU_core_FOM         *FOM);
float* IMU_core_estmAccl(
  unsigned short              id, 
  float                       t, 
  float                       *a, 
  struct IMU_core_FOM         *FOM);
float* IMU_core_estmMagn(
  unsigned short              id, 
  float                       t, 
  float                       *m, 
  struct IMU_core_FOM         *FOM);
float* IMU_core_estmAll(
  unsigned short              id, 
  float                       t, 
  float                       *g, 
  float                       *a, 
  float                       *m,  
  struct IMU_core_FOM         FOM[3]);
float* IMU_core_estmMove(
  unsigned short              id);


#ifdef __cplusplus
}
#endif

#endif

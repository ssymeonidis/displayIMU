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

// define the configuration structure (values tuned for a part)
struct IMU_core_config {
  unsigned char  isGyro;          // enable gyroscope data
  unsigned char  isAccl;          // enable accelerometer data
  unsigned char  isMagn;          // enable magnetometer data
  unsigned char  isFltr;          // enable IMU weight 
  unsigned char  isTear;          // enable application of ref
  unsigned char  isMove;          // enable acceleration estimate
  unsigned char  isFOM;           // enable FOM calculation
  unsigned char  isAutocal;       // enable autocal data collect
  float          gThreshVal;      // no motion threshold value
  float          gThreshTime;     // no motion threhsold time
  float          aWeight;         // accelerometer IMU weight
  float          aAlpha;          // accelerometer filter weight 
  float          mWeight;         // magnetometer IMU weight
  float          mAlpha;          // magnetometer filter weight 
  float          moveAlpha;       // acceleration estimate alpha
  float          autocalAlpha;    // autocal "still" alpha
};

// define internal state struct (captures internal IMU state)
struct IMU_core_state {
  float          SEq[4];          // current quaterion
  float          ref[4];          // "tear" reference 
  float          a[3];            // last accelerometer input
  float          A[3];            // last acceleration estimate
  unsigned char  isReset;         // reset signal
};

// define the quality metrics structure (used for debugging)
struct IMU_core_metrics {
  float          delta_G;
  float          delta_a;
  float          delta_m;
  float          delta_M;
  float          delta_ang;
};

// data structure access functions
void IMU_core_getConfig   (struct IMU_core_config  **config);
void IMU_core_getState    (struct IMU_core_state   **state);

// general operation functions 
#define IMU_CORE_ESTM_ARGS float* E, float* A, struct IMU_core_metrics* FOM 
void IMU_core_init        ();
void IMU_core_deadRecon   (float* a, float* m, float* E);
void IMU_core_estmGyro    (float* t, float* g, IMU_CORE_ESTM_ARGS);
void IMU_core_estmAccl    (float* t, float* a, IMU_CORE_ESTM_ARGS);
void IMU_core_estmMagn    (float* t, float* m, IMU_CORE_ESTM_ARGS);
void IMU_core_estmAll     (float* t, float* g, float* a, float* m,  
                           IMU_CORE_ESTM_ARGS);

#ifdef __cplusplus
}
#endif

#endif

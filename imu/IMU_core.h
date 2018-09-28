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

#ifndef _DISPLAYIMU_H
#define _DISPLAYIMU_H

#ifdef __cplusplus
extern "C" {
#endif

// define the configuration structure (values tuned for a part)
struct displayIMU_config {
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
struct displayIMU_state {
  float          SEq[4];          // current quaterion
  float          ref[4];          // "tear" reference 
  float          a[3];            // last accelerometer input
  float          A[3];            // last acceleration estimate
  unsigned char  isReset;         // reset signal
};

// define the quality metrics structure (used for debugging)
struct displayIMU_metrics {
  float          delta_G;
  float          delta_a;
  float          delta_m;
  float          delta_M;
  float          delta_ang;
};

// data structure access functions
void displayIMU_getConfig   (struct displayIMU_config  **config);
void displayIMU_getState    (struct displayIMU_state   **state);

// "tear" functions
void displayIMU_setRef      ();
void displayIMU_setRefAccl  (float* a);

// general operation functions 
#define displayIMU_ESTM_ARGS float* E, float* A, struct displayIMU_metrics* FOM 
void displayIMU_init        ();
void displayIMU_deadRecon   (float* a, float* m, float* E);
void displayIMU_estmGyro    (float* t, float* g, displayIMU_ESTM_ARGS);
void displayIMU_estmAccl    (float* t, float* a, displayIMU_ESTM_ARGS);
void displayIMU_estmMagn    (float* t, float* m, displayIMU_ESTM_ARGS);
void displayIMU_estmAll     (float* t, float* g, float* a, float* m,  
                             displayIMU_ESTM_ARGS);

#ifdef __cplusplus
}
#endif

#endif

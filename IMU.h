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

// define the calibration structure (values tuned for each unit)
struct displayIMU_calib {
  float    gBias[3];        // gyroscope biases
  float    gMult[9];        // gyroscope transform matrix 
  float    aBias[3];        // accelerometer biases
  float    aMult[9];        // accelerometer transform matrix 
  float    mBias[3];        // magnetometer biases
  float    mMult[9];        // magnetometer transform matrix
  float    aMag;            // gravity magnitude
  float    mMag;            // magnetic north magnitude  
  float    mAng;            // magnetic north angle
};

// define the configuration structure (values tuned for a part)
struct displayIMU_config {
  bool     isGyro;          // enable gyroscope data
  bool     isAccl;          // enable accelerometer data
  bool     isMagn;          // enable magnetometer data
  bool     isWeight;        // enable IMU weight 
  bool     isTear;          // enable application of ref
  bool     isMove;          // enable acceleration estimate
  bool     isFOM;           // enable FOM calculation
  bool     isFltr;          // enable "bad data" filtering 
  bool     isAutocal;       // enable autocal data collect
  float    gThreshVal;      // no motion threshold value
  float    gThreshTime;     // no motion threhsold time
  float    aWeight;         // accelerometer IMU weight
  float    aAlpha;          // accelerometer filter weight 
  float    mWeight;         // magnetometer IMU weight
  float    mAlpha;          // magnetometer filter weight 
  float    moveAlpha;       // acceleration estimate alpha
  float    autocalAlpha;    // autocal "still" alpha
};

// define internal state struct (captures internal IMU state)
struct displayIMU_state {
  float    SEq[4];          // current quaterion
  float    ref[4];          // "tear" reference 
  float    a[3];            // last accelerometer input
  float    A[3];            // last acceleration estimate
  bool     isReset;         // reset signal
};

// define the auto_calib state (used to improve config struct)
struct displayIMU_autocal {
  float    gBias[3];        // gyroscope autocal estimate #1
  float    gBiasCont[3];    // gyroscope autocal estimate #2
  float    aMag;            // gravity magnitude estimate #1
  float    aMagCont;        // gravity magnitude estimate #2
  float    mMag;            // magnetic north magnitude estimate #1
  float    mMagCont;        // magnetic north magnitude estimate #2
  float    mAng;            // magnetic north angle estimate #1
  float    mAngCont;        // magnetic north angle estiamte #2
}; 

// define the quality metrics structure (used for debugging)
struct displayIMU_metrics {
  float    delta_G;
  float    delta_a;
  float    delta_m;
  float    delta_M;
  float    delta_ang;
};

// data structure access functions
void displayIMU_getCalib    (displayIMU_calib   **calib);
void displayIMU_getConfig   (displayIMU_config  **config);
void displayIMU_getState    (displayIMU_state   **state);
void displayIMU_getAutocal  (displayIMU_autocal *autocal);

// "tear" functions
void displayIMU_setRef      ();
void displayIMU_setRefAccl  (float* a);

// raw data correction functions
void displayIMU_corGyro     (float* g_raw, float* g);
void displayIMU_corAccl     (float* a_raw, float* a);
void displayIMU_corMagn     (float* m_raw, float* m);
void displayIMU_corAll      (float* g_raw, float* a_raw, float* m_raw, 
                             float* g,     float* a,     float* m);

// general operation functions 
#define displayIMU_ESTM_ARGS float* E, float* A, displayIMU_metrics* FOM 
void displayIMU_init        ();
void displayIMU_deadRecon   (float* a, float* m, float* E);
void displayIMU_estmGyro    (float* t, float* g, displayIMU_ESTM_ARGS);
void displayIMU_estmAccl    (float* t, float* a, displayIMU_ESTM_ARGS);
void displayIMU_estmMagn    (float* t, float* m, displayIMU_ESTM_ARGS);
void displayIMU_estmAll     (float* t, float* g, float* a, float* m,  
                             displayIMU_ESTM_ARGS);

#endif

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

#ifndef _IMU_CALIB_PNTS_H
#define _IMU_CALIB_PNTS_H

#ifdef __cplusplus
extern "C" {
#endif

// define error codes
#define IMU_CALIB_PNTS_INST_OVERFLOW -1
#define IMU_CALIB_PNTS_BAD_INST      -2
#define IMU_CALIB_PNTS_EMPTY_TABLE   -3


// define the configuration structure (values tuned for a part)
struct IMU_calib_pnts_config {
  unsigned char       isAccl;
  unsigned char       isMagn;
  float               gThreshVal;      // no motion threshold value
  float               gThreshTime;     // no motion threhsold time
  float               gAlpha;
  float               aAlpha;          // accelerometer filter value
  float               mAlpha;          // magnetometer filter value
};

// define internal state struct (captures internal IMU state)
struct IMU_calib_pnts_state {
  int                 index;
  float               t;               // last datum time
  float               tStable;         // last "unstable" time
  float               gAccum[3];       // 
  float               gFltr[3];
  float               aFltr[3];
  float               gFltr[3];
  unsigned char       stable;
  unsigned char       tReset;
  unsigned char       gCurrent;
  unsigned char       aReset;
  unsigned char       aCurrent;
  unsigned char       mReset;
  unsigned char       mCurrent;
};

// define 
struct IMU_calib_pnts_entry {
  float               tStart;
  float               tEnd;
  float               gAccum[3];
  float               gFltr[3];
  float               aFltr[3]; 
  float               gFltr[3];
};


// data structure access functions
int IMU_calib_pnts_init(
  unsigned short                  *id, 
  struct IMU_calib_pnts_config    **config);
int IMU_calib_pnts_getState(
  unsigned short                  id,  
  struct IMU_calib_pnts_state     **state);
int IMU_calib_pnts_getEntry(
  unsigned short                  id,
  struct IMU_calib_pnts_entry     **entry);
int IMU_calib_pnts_reset(
  unsigned short                  id);

// general operation functions 
void IMU_calib_pnts_updateGyro(
  unsigned short                  id, 
  float                           t, 
  float                           *g); 
void IMU_calib_pnts_updateAccl(
  unsigned short                  id, 
  float                           t, 
  float                           *a); 
void IMU_calib_pnts_updateMagn(
  unsigned short                  id, 
  float                           t, 
  float                           *m); 
void IMU_calib_pnts_updateAll(
  unsigned short                  id,
  float                           t,
  float                           *g,
  float                           *a,
  float                           *m);


#ifdef __cplusplus
}
#endif

#endif

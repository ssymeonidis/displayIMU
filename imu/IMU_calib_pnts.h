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
  unsigned char          isAccl;          // process accelerometer data
  unsigned char          isMagn;          // process magnetometer data
  float                  gAlpha;          // mean/std calc filter value 
  float                  gThreshVal;      // no motion threshold value
  float                  gThreshTime;     // no motion threhsold time
  float                  aAlpha;          // accelerometer filter value
  float                  aThresh;         // no motion threshold value
  float                  mAlpha;          // magnetometer filter value
  float                  mThresh;         // no motion threshold value
};

// internal state for IMU_calib_pnts functions
enum calib_pnts_state {
  stop, 
  reset,
  moving,
  stable
};

// define internal state struct (captures internal IMU state)
struct IMU_calib_pnts_state {
  unsigned short         numPnts;
  unsigned short         curPnts;
  enum calib_pnts_state  state;
  unsigned short         index;
  float                  tStable;
  unsigned char          aClock;
  unsigned char          mClock;
  float                  gMean[3];
};

// define 
struct IMU_calib_pnts_entry {
  float                  tStart;
  float                  tEnd;
  float                  gAccum[3];
  float                  gFltr[3];
  float                  aFltr[3]; 
  float                  mFltr[3];
};


// data structure access function
int IMU_calib_pnts_init(
  unsigned short                  *id, 
  struct IMU_calib_pnts_config    **config);
int IMU_calib_pnts_getState(
  unsigned short                  id,  
  struct IMU_calib_pnts_state     **state);
int IMU_calib_pnts_getCount(
  unsigned short                  id,
  unsigned short                  *count);
int IMU_calib_pnts_start(
  unsigned short                  id,
  unsigned short                  numPnts);
int IMU_calib_pnts_stop(
  unsigned short                  id);

// points table access function
#if IMU_CALIB_TABLE_SIZE > 1
int IMU_calib_pnts_getEntry(
  unsigned short                  id,
  unsigned short                  index,
  struct IMU_calib_pnts_entry     **entry);
#endif

// general operation functions 
int IMU_calib_pnts_updateGyro(
  unsigned short                  id, 
  float                           t, 
  float                           *g,
  struct IMU_calib_pnts_entry     **entry); 
int IMU_calib_pnts_updateAccl(
  unsigned short                  id, 
  float                           t, 
  float                           *a,
  struct IMU_calib_pnts_entry     **entry); 
int IMU_calib_pnts_updateMagn(
  unsigned short                  id, 
  float                           t, 
  float                           *m, 
  struct IMU_calib_pnts_entry     **entry); 
int IMU_calib_pnts_updateAll(
  unsigned short                  id,
  float                           t,
  float                           *g,
  float                           *a,
  float                           *m,
  struct IMU_calib_pnts_entry     **entry); 


#ifdef __cplusplus
}
#endif

#endif

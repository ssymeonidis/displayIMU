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

#ifndef _IMU_CALIB_CTRL_H
#define _IMU_CALIB_CTRL_H

#include "IMU_calib_pnts.h"
#include "IMU_core.h"
#include "IMU_correct.h"

#ifdef __cplusplus
extern "C" {
#endif

// define status codes
#define IMU_CALIB_CTRL_UPDATED        1

// define error codes
#define IMU_CALIB_CTRL_INST_OVERFLOW -1
#define IMU_CALIB_CTRL_BAD_INST      -2
#define IMU_CALIB_CTRL_BAD_MODE      -3


// define calibration types
enum IMU_calib_ctrl_mode {
  IMU_calib_ctrl_NA               = -1,
  IMU_calib_ctrl_4pnt             = 0,
  IMU_calib_ctrl_6pnt             = 1
};

// define internal state
struct IMU_calib_ctrl_state {
  enum IMU_calib_ctrl_mode        mode;
  struct IMU_correct_config       correct;
  struct IMU_core_config          core;
  struct IMU_calib_pnts_entry     table[IMU_CALIB_CTRL_SIZE]; 
  unsigned short                  numPnts;
};

// define figure of merit
struct IMU_calib_ctrl_FOM {
  float                           empty;
};


// general operation functions
int IMU_calib_ctrl_init(
  unsigned short                  *id);
int IMU_calib_ctrl_start(
  unsigned short                  id,
  enum IMU_calib_ctrl_mode         mode,
  struct IMU_correct_config       *correct,
  struct IMU_core_config          *core);
int IMU_calib_ctrl_update(
  unsigned short                  id, 
  struct IMU_calib_pnts_entry     *pnt,
  struct IMU_calib_ctrl_FOM       *FOM);
int IMU_calib_ctrl_save(
  unsigned short                  id,
  struct IMU_correct_config       *correct);


#ifdef __cplusplus
}
#endif

#endif

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

#ifndef _IMU_UTIL_FILE_H
#define _IMU_UTIL_FILE_H

// include statements
#include "IMU_core.h"
#include "IMU_correct.h"
#include "IMU_calib_pnts.h"

#ifdef __cplusplus
extern "C" {
#endif

// define error codes
#define IMU_UTIL_FILE_INVALID_FILE   -1
#define IMU_UTIL_FILE_INVALID_FIELD  -2
#define IMU_UTIL_FILE_UNEXPECTED_EOF -3
#define IMU_UTIL_FILE_MISSING_ARGS   -4
#define IMU_UTIL_FILE_INVALID_BOOL   -5


// functions used to create custom json reader 
int IMU_util_getLine  (FILE *file, char** field, char** args);
int IMU_util_getField (char* field, const char* names[], int size);

// functions to access imu specfic json readers and writers 
int IMU_util_readCorrect(
  char*                         filename, 
  struct IMU_correct_config     *config);
int IMU_util_writeCorrect(
  char*                         filename, 
  struct IMU_correct_config     *config);
int IMU_util_readCore(
  char*                         filename, 
  struct IMU_core_config        *config);
int IMU_util_writeCore(
  char*                         filename, 
  struct IMU_core_config        *config);
int IMU_util_readCalibPnts(
  char*                         filename,
  struct IMU_calib_pnts_config  *config);
int IMU_util_writeCalibPnts(
  char*                         filename,
  struct IMU_calib_pnts_config  *config);



#ifdef __cplusplus
}
#endif

#endif

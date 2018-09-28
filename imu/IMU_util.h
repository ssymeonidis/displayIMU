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

#ifndef _IMU_UTIL_H
#define _IMU_UTIL_H

// include statements
#include "IMU_core.h"
#include "IMU_correct.h"

#ifdef __cplusplus
extern "C" {
#endif

// functions to read/write sturctures to json files 
int IMU_util_getLine     (FILE *file, char** field, char** args);
int IMU_util_getField    (char* field, const char* names[], int size);
int IMU_util_readCalib   (char* filename, struct IMU_correct_calib *calib);
int IMU_util_writeCalib  (char* filename, struct IMU_correct_calib *calib);
int IMU_util_readConfig  (char* filename, struct IMU_core_config *config);
int IMU_util_writeConfig (char* filename, struct IMU_core_config *config);

#ifdef __cplusplus
}
#endif

#endif

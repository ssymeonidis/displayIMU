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

#ifndef _IMU_FILE_H
#define _IMU_FILE_H

// include statements
#include "IMU_rect.h"
#include "IMU_core.h"
#include "IMU_pnts.h"
#include "IMU_auto.h"

#ifdef __cplusplus
extern "C" {
#endif

// define error codes
#define IMU_FILE_INVALID_FILE   -1
#define IMU_FILE_INVALID_FIELD  -2
#define IMU_FILE_UNEXPECTED_EOF -3
#define IMU_FILE_MISSING_ARGS   -4
#define IMU_FILE_INVALID_BOOL   -5


// functions used to create custom json reader 
int IMU_file_getLine  (FILE *file, char **field, char **args);
int IMU_file_getField (char *field, const char *names[], int size);

// functions to access imu specfic json readers and writers 
int IMU_file_readRect  (char *filename, IMU_rect_config *config);
int IMU_file_writeRect (char *filename, IMU_rect_config *config);
int IMU_file_readCore  (char *filename, IMU_core_config *config);
int IMU_file_writeCore (char *filename, IMU_core_config *config);
int IMU_file_readPnts  (char *filename, IMU_pnts_config *config);
int IMU_file_writePnts (char *filename, IMU_pnts_config *config);
int IMU_file_readAuto  (char *filename, IMU_auto_config *config);
int IMU_file_writeAuto (char *filename, IMU_auto_config *config);


#ifdef __cplusplus
}
#endif

#endif

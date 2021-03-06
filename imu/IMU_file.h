/*
 * This file is part of quaternion-based displayIMU C/C++/QT code base
 * (https://github.com/ssymeonidis/displayIMU.git)
 * Copyright (c) 2018 Simeon Symeonidis (formerly Sensor Management Real
 * Time (SMRT) Processing Solutions)
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
#include "IMU_stat.h"
#include "IMU_calb.h"
#include "IMU_engn.h"

#ifdef __cplusplus
extern "C" {
#endif

// define error codes
#define IMU_FILE_INVALID_FILE   -1
#define IMU_FILE_INVALID_FIELD  -2
#define IMU_FILE_UNEXPECTED_EOF -3
#define IMU_FILE_MISSING_ARGS   -4
#define IMU_FILE_INVALID_BOOL   -5


// functions to access imu specfic json readers and writers 
int IMU_file_coreLoad (const char *filename, IMU_core_config *config);
int IMU_file_coreSave (const char *filename, IMU_core_config *config);
int IMU_file_rectLoad (const char *filename, IMU_rect_config *config);
int IMU_file_rectSave (const char *filename, IMU_rect_config *config);
int IMU_file_pntsLoad (const char *filename, IMU_pnts_config *config);
int IMU_file_pntsSave (const char *filename, IMU_pnts_config *config);
int IMU_file_statLoad (const char *filename, IMU_stat_config *config);
int IMU_file_statSave (const char *filename, IMU_stat_config *config);
int IMU_file_calbLoad (const char *filename, IMU_calb_config *config);
int IMU_file_calbSave (const char *filename, IMU_calb_config *config);
int IMU_file_engnLoad (const char *filename, IMU_engn_config *config);
int IMU_file_engnSave (const char *filename, IMU_engn_config *config);


#ifdef __cplusplus
}
#endif

#endif

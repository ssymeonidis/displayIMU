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

#ifndef _IMU_CALIB_CTRL_H
#define _IMU_CALIB_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

// define error codes
#define IMU_CALIB_CTRL_INST_OVERFLOW -1
#define IMU_CALIB_CTRL_BAD_INST      -2


// data structure access functions
int IMU_calib_ctrl_init(unsigned short *id);

// general operation functions
int IMU_calib_ctrl_4pnt_init   (unsigned short id);
int IMU_calib_ctrl_4pnt_update (unsigned short id, IMU_calib_pnts_entry *pnt);
int IMU_calib_ctrl_4pnt_save   (unsigned short id);
int IMU_calib_ctrl_6pnt_init   (unsigned short id);
int IMU_calib_ctrl_6pnt_update (unsigned short id, IMU_calib_pnts_entry *pnt);
int IMU_calib_ctrl_6pnt_save   (unsigned short id);


#ifdef __cplusplus
}
#endif

#endif

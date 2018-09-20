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

#ifndef _RECEIVE_H
#define _RECEIVE_H

// include statements
#include "MARG.h"

// define the sensor data structure
extern const int           sensor_buffer_size;
extern float               sensor_buffer[][15];
extern int                 sensor_buffer_index;
extern displayIMU_metrics  sensor_buffer_metrics;
extern int                 sensor_IMU_reset;
extern int                 sensor_IMU_set_ref;
extern int                 sensor_IMU_calib;

void sensor_data_error(const char *msg);
void sensor_data_init(int portno);
void *sensor_data_run(void* id);

#endif

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

#ifndef _DATAREAD_H
#define _DATAREAD_H

// include statemeents
#include "MARG.h"

// define the sensor data structure
extern const int           csv_buffer_size;
extern float               csv_buffer[][16];
extern int                 csv_buffer_index;
extern displayIMU_metrics  csv_buffer_metrics;
extern int                 csv_IMU_set_ref;
extern int                 csv_IMU_reset;
extern int                 csv_IMU_calib;

void csv_data_init(char* filename);
void *csv_data_run(void* id);

#endif

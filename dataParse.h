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
#include "IMU.h"

// define the sensor data structure
extern const int           buffer_size;
extern float               buffer[][15];
extern int                 buffer_index;
extern displayIMU_metrics  FOM;

// access functions
void data_init_log(const char* filename);
void data_init_UDP(int portno);
void data_init_CSV(const char* filename);
void *data_run(void* id);

#endif

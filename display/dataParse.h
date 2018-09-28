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

#ifndef _DATA_PARSE_H
#define _DATA_PARSE_H

// include statements
#include "IMU_core.h"
#include "IMU_correct.h"

// define input sensor data structure
struct dataParse_sensor {
  float gyroRaw[3];
  float gyroCor[3];
  float gyroFltr[3];
  float acclRaw[3];
  float acclCor[3];
  float acclFltr[3];
  float magnRaw[3];
  float magnCor[3];
  float magnFltr[3];
  float lastTime;
}; 

// define IMU estimate data structure
struct dataParse_estim {
  float ang[3];
  float move[3];  
};

// define the sensor data structure
extern dataParse_sensor    sensor;
extern dataParse_estim     estim;
extern displayIMU_metrics  FOM;

// access functions
void data_init_log(const char* filename);
void data_init_UDP(int portno);
void data_init_CSV(const char* filename);
void *data_run(void* id);

#endif

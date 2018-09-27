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

#ifndef FILE_UTILS_H
#define FILE_UTILS_H

// include statements
#include "IMU.h"

// functions to read/write sturctures to json files 
int displayIMU_getLine(FILE *file, char** field, char** args);
int displayIMU_readConfig(char* filename, displayIMU_config *config);
int displayIMU_writeConfig(char* filename, displayIMU_config *config);
int displayIMU_readCalib(char* filename, displayIMU_calib *calib);
int displayIMU_writeCalib(char* filename, displayIMU_calib *calib);

#endif

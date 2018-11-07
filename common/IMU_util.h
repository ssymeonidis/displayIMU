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

#ifndef _IMU_UTILS_H
#define _IMU_UTILS_H

// include statements
#include <stdio.h>

// function definitions
void IMU_util_status  (int status, const char  *message);


/******************************************************************************
* checks status and prints error message if necessary
******************************************************************************/

void IMU_util_status(
  int                      status, 
  const char               *message)
{
  if (status < 0) {
    printf("error: %s :%d\n", message, status);
    exit(0);
  }
}


#endif

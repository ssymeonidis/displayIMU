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

#ifndef _TEST_UTILS_H
#define _TEST_UTILS_H

// include statements
#include <math.h>

// define constants
static const int   msg_delay   = 200;
static const float precision   = 0.05;

// function definitions
void check_state   (int status,     char  *message);
void verify_data   (float val1,     float val);
void verify_vect   (float val1[3],  float val2[3]);
void verify_quat   (float val1[4],  float val2[4]);


/******************************************************************************
* checks status and prints error message if necessary
******************************************************************************/

void check_status(
  int                      status, 
  char                     *message)
{
  if (status < 0) {
    printf("error: %s :%d\n", message, status);
    exit(0);
  }
}


/******************************************************************************
* verifies vector against intended result
******************************************************************************/

void verify_data(
  float                val1,
  float                val2)
{
  if (fabs(val1 - val2) > precision) {
    printf("error: vect results failure\n");
    exit(0);
  }
}


/******************************************************************************
* verifies vector against intended result
******************************************************************************/

void verify_vect(
  float                val1[3],
  float                val2[3])
{
  if (fabs(val1[0] - val2[0]) > precision ||
      fabs(val1[1] - val2[1]) > precision ||
      fabs(val1[2] - val2[2]) > precision) {
    printf("error: vect results failure\n");
    exit(0);
  }
}


/******************************************************************************
* verifies quaternion against intended result
******************************************************************************/

void verify_quat(
  float                val1[3],
  float                val2[3])
{
  if (fabs(val1[0] - val2[0]) > precision ||
      fabs(val1[1] - val2[1]) > precision ||
      fabs(val1[2] - val2[2]) > precision ||
      fabs(val1[3] - val2[3]) > precision) {
    printf("error: quat results failure\n");
    exit(0);
  }
}


#endif

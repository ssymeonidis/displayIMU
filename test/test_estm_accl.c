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

// include statements
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "IMU_math.h"

// define constants
static const float precision   = 0.05;

// internal functions
static void print_line    (float FOM, float q[4]);
static void verify_quat   (float val1[4], float val2[4]);


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // define internal constants
  const float alpha    = 0.01;
  const int   iter     = 70;
  
  // define internal variables
  float FOM;
  int   i;
  
  // start datum test
  printf("starting test_estm_accl...\n");
  
  // test zero-pitch, zero-roll
  float q_1[4]   = {0.8552, 0.3665, 0.3665, 0.0};
  float ref_1[4] = {1.0, 0.0, 0.0, 0.0};
  float a_1[3]   = {0.0, 0.0, 1.0};
  for (i=0; i<iter; i++) {
    IMU_math_estmAccl(q_1, a_1, alpha, &FOM);
    print_line(FOM, q_1);
  }
  if (FOM > 0.01) {
    printf("error: FOM convergance greater that threshold\n");
    exit(0);
  }
  verify_quat(q_1, ref_1);

  // exit program
  printf("pass: test_estm_accl\n\n");
  return 0;
}


/******************************************************************************
* prints results to console
******************************************************************************/

void print_line(
  float                FOM,
  float                q[4])
{
  // start datum test
  printf("%0.3f, %0.2f, %0.2f, %0.2f, %0.2f\n", FOM, q[0], q[1], q[2], q[3]);
}


/******************************************************************************
* verifies euler angle against intended result
******************************************************************************/

void verify_quat(
  float                val1[3],
  float                val2[3])
{
  if (fabs(val1[0] - val2[0]) > precision ||
      fabs(val1[1] - val2[1]) > precision ||
      fabs(val1[2] - val2[2]) > precision ||
      fabs(val1[3] - val2[3]) > precision) {
    printf("error: quaternion accuracy failure\n");
    exit(0);
  }
}

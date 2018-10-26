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
#include "test_utils.h"

// internal functions
static void print_line    (float FOM, float q[4]);


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
  printf("starting test_estm_magn...\n");
  
  // test zero-pitch, zero-roll
  float q_1[4]   = {0.8552, 0.3665, 0.3665, 0.0};
  float ref_1[4] = {1.0, 0.0, 0.0, 0.0};
  float m_1[3]   = {1.0, 0.0, 0.0};
  for (i=0; i<iter; i++) {
    IMU_math_estmMagnRef(q_1, m_1, 0.0, 1.0, alpha, &FOM);
    print_line(FOM, q_1);
  }
  if (FOM > 0.01) {
    printf("error: FOM convergance greater that threshold\n");
    exit(0);
  }
  verify_quat(q_1, ref_1);

  // exit program
  printf("pass: test_estm_magn\n\n");
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

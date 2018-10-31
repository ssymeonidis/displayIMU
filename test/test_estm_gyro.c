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
#include "test_utils.h"
#include "IMU_math.h"

// internal functions
static void process_rotate (float q[3], float gyro[3], float dt, 
                            int num_iter, float q_ref[4]);


/******************************************************************************
* main function - simple test of gyroscope core
******************************************************************************/

int main(void)
{
  // start datum test
  printf("starting test_estm_gyro...\n");
  float dt        = 0.1;
  int   num_iter  = 60;
  
  // test roll
  float q1[4]     = { 1.0000,  0.0000,  0.0000,  0.0000};
  float gyro1[3]  = { 0.2618,  0.0000,  0.0000};
  float ref1[4]   = { 0.7108,  0.7108,  0.0000,  0.0000};
  process_rotate(q1, gyro1, dt, num_iter, ref1);

  // test pitch
  float q2[4]     = { 1.0000,  0.0000,  0.0000,  0.0000};
  float gyro2[3]  = { 0.0000,  0.2618,  0.0000};
  float ref2[4]   = { 0.7108,  0.0000,  0.7108,  0.0000};
  process_rotate(q2, gyro2, dt, num_iter, ref2);

  // test rotate
  float q3[4]     = { 1.0000,  0.0000,  0.0000,  0.0000};
  float gyro3[3]  = { 0.0000,  0.0000,  0.2618};
  float ref3[4]   = { 0.7108,  0.0000,  0.0000,  0.7108};
  process_rotate(q3, gyro3, dt, num_iter, ref3);

  // exit program
  printf("pass: test_estm_gyro\n\n");
  return 0;
}


/******************************************************************************
* prints results to console
******************************************************************************/

void process_rotate(
  float                q[3],
  float                gyro[3],
  float                dt,
  int                  num_iter,
  float                q_ref[4])
{
  // main processing loop
  int                  i;
  for (i=0; i<num_iter; i++)
    IMU_math_estmGyro(q, gyro, dt);

  // print and verifiy final quaternion
  printf("%0.2f, %0.2f, %0.2f, %0.2f\n", q[0], q[1], q[2], q[3]);
  verify_quat(q, q_ref);
}

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

// define internal constants
const float precision_ang = 7.0;

// internal functions
static void process_angles (float ang[3], float vec[3], float out[3]);


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // start datum test
  printf("starting test_estm_magn...\n");
  
  // test zero yaw
  float ang1[3]   = {25.0, 15.0, 15.0};
  float vec1[3]   = { 1.0,  0.0,  0.0};
  float out1[3]   = { 0.0,  0.0,  0.0};
  process_angles(ang1, vec1, out1);
  if (fabs(out1[0]) >  precision_ang) {
    printf("error: failed zero yaw\n");
    exit(0);
  }

  // test 90deg yaw
  float ang2[3]   = {65.0, 15.0, 15.0};
  float vec2[3]   = { 0.0, -1.0,  0.0};
  float out2[3]   = { 0.0,  0.0,  0.0};
  process_angles(ang2, vec2, out2);
  if (fabs(out2[0] - 90.0) >  precision_ang) {
    printf("error: failed 90deg yaw\n");
    exit(0);
  }

  // test neg90deg yaw
  float ang3[3]   = {-65.0, 15.0, 15.0};
  float vec3[3]   = { 0.0,  1.0,  0.0};
  float out3[3]   = { 0.0,  0.0,  0.0};
  process_angles(ang3, vec3, out3);
  if (fabs(out3[0] + 90.0) > precision_ang) {
    printf("error: failed neg90deg yaw\n");
    exit(0);
  }

  // test 180deg yaw
  float ang4[3]   = {-205.0, 15.0, 15.0};
  float vec4[3]   = {-1.0,  0.0,  0.0};
  float out4[3]   = { 0.0,  0.0,  0.0};
  process_angles(ang4, vec4, out4);
  if (fabs(out4[0] + 180.0) > precision_ang &&
      fabs(out4[0] - 180.0) > precision_ang) {
    printf("error: failed 180deg yaw\n");
    exit(0);
  }

  // exit program
  printf("pass: test_estm_magn\n\n");
  return 0;
}


/******************************************************************************
* prints results to console
******************************************************************************/

void process_angles(
  float                ang[3],
  float                vec[3],
  float                out[3])
{
  // define local constants
  const float alpha    = 0.005;
  const int   iter     = 100;

  // define local variables
  float                radians[3];
  float                q[3];
  float                FOM;
  int                  i;

  // main processing loop
  IMU_math_degToRad(ang, radians);
  IMU_math_eulerToQuat(radians, q);
  for (i=0; i<iter; i++)
    IMU_math_estmMagnNorm(q, vec, alpha, &FOM);
  IMU_math_quatToEuler(q, radians);
  IMU_math_radToDeg(radians, out);
  printf("%0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n", 
    ang[0], ang[1], ang[2], vec[0], vec[1], vec[2], out[0], out[1], out[2]);
}

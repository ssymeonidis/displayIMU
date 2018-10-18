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
static void test_quat     (float quat[4],  float euler[3]); 
static void print_line    (float input[4], float result[3], float quat[4]);
static void verify_euler  (float val1[3],  float val2[3]);
static void verify_quat   (float val1[4],  float val2[4]);


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // start datum test
  printf("starting test_euler...\n");
  
  // testing value
  float input1[4] = {1, 0, 0, 0};
  float test1[3]  = {0, 0, 0};
  test_quat(input1, test1);

  // testing value
  float input2[4] = {0, 1, 0, 0};
  float test2[3]  = {0, 0, 180};
  test_quat(input2, test2);

  // testing value
  float input3[4] = {0, 0, 1, 0};
  float test3[3]  = {180, 0, 180};
  test_quat(input3, test3);

  // testing value
  float input4[4] = {0, 0, 0, 1};
  float test4[3]  = {180, 0, 0};
  test_quat(input4, test4);

  // testing value
  float input5[4] = {sqrt(0.5), sqrt(0.5), 0, 0};
  float test5[3]  = {0, 0, 90};
  test_quat(input5, test5);

  // testing value
  float input6[4] = {sqrt(0.5), 0, sqrt(0.5), 0};
  float test6[3]  = {0, 90, 0};
  test_quat(input6, test6);

  // testing value
  float input7[4] = {sqrt(0.5), 0, 0, sqrt(0.5)};
  float test7[3]  = {90, 0, 0};
  test_quat(input7, test7);

  // testing value
  float input8[4] = {sqrt(0.5), -sqrt(0.5), 0, 0};
  float test8[3]  = {0, 0, -90};
  test_quat(input8, test8);

  // testing value
  float input9[4] = {sqrt(0.5), 0, -sqrt(0.5), 0};
  float test9[3]  = {0, -90, 0};
  test_quat(input9, test9);

  // testing value
  float input10[4] = {sqrt(0.5), 0, 0, -sqrt(0.5)};
  float test10[3]  = {-90, 0, 0};
  test_quat(input10, test10);

  // exit program
  printf("pass: test_euler\n\n");
  return 0;
}


/******************************************************************************
* prints results to console
******************************************************************************/

void test_quat(
  float                input[4],  
  float                euler[3])
{
  // testing value
  float test[3] = {0, 0, 0};
  float temp[3] = {0, 0, 0};
  float quat[4] = {0, 0, 0, 0};
  IMU_math_quatToEuler(input, test);
  IMU_math_radToDeg(test, test);
  IMU_math_degToRad(test, temp);
  IMU_math_eulerToQuat(temp, quat);
  print_line(input, test, quat);
  verify_euler(test, euler);
  verify_quat(quat, input);
}


/******************************************************************************
* prints results to console
******************************************************************************/

void print_line(
  float                input[4],
  float                euler[3],
  float                quat[4])
{
  // start datum test
  printf("%0.2f, %0.2f, %0.2f, %0.2f, %0.3f, %0.3f, %0.3f, ",
    input[0], input[1], input[2], input[3], euler[0], euler[1], euler[2]);
  printf("%0.2f, %0.2f, %0.2f, %0.2f\n",
    quat[0], quat[1], quat[2], quat[3]);
}


/******************************************************************************
* verifies euler angle against intended result
******************************************************************************/

void verify_euler(
  float                val1[3],
  float                val2[3])
{
  if (fabs(val1[0] - val2[0]) > precision ||
      fabs(val1[1] - val2[1]) > precision ||
      fabs(val1[2] - val2[2]) > precision) {
    printf("error: euler results failure\n");
    exit(0);
  }
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
    printf("error: euler results failure\n");
    exit(0);
  }
}

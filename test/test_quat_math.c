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


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // start datum test
  printf("starting test_quat_math...\n");
  
  // testing quaternion multiply
  float q1a[4]  = { 0.9143, -0.0292,  0.6006, -0.7162};
  float q1b[4]  = {-0.1565,  0.8315,  0.5844,  0.9190};
  float ans1[4] = { 0.1885,  1.7353, -0.1283,  0.4359};
  float out1[4] = { 0.0000,  0.0000,  0.0000,  0.0000};
  IMU_math_quatMult(q1a, q1b, out1);
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", out1[0], out1[1], out1[2], out1[3]);
  verify_quat(ans1, out1);

  // testing quaternion multiply conjugate
  float q2a[4]  = { 0.3110, -0.6576,  0.4121, -0.9363};
  float q2b[4]  = {-0.4462, -0.9077, -0.8057,  0.6469};
  float ans2[4] = {-0.4796,  1.0635, -1.2086, -0.6873};
  float out2[4] = { 0.0000,  0.0000,  0.0000,  0.0000};
  IMU_math_quatMultConj(q2a, q2b, out2);
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", out2[0], out2[1], out2[2], out2[3]);
  verify_quat(ans2, out2);

  // testing quaternion vector rotate forward
  float q3[4]   = {-0.4700, -0.1403, -0.0726, -0.8684};
  float v3[3]   = { 0.3897, -0.3685,  0.9004};
  float ans3[3] = { 0.3700,  0.5211,  0.8295};
  float out3[3] = { 0.0000,  0.0000,  0.0000};
  IMU_math_rotateForward(v3, q3, out3);
  printf("%0.3f, %0.3f, %0.3f\n", out3[0], out3[1], out3[2]);
  verify_vect(ans3, out3);

  // testing quaternion vector rotate reverse
  float q4[4]   = {-0.4128, -0.2261, -0.2953, -0.8314};
  float v4[3]   = {-0.9311, -0.1225, -0.2369};
  float ans4[3] = { 0.3868,  0.4137, -0.7857};
  float out4[3] = { 0.0000,  0.0000,  0.0000};
  IMU_math_rotateReverse(v4, q4, out4);
  printf("%0.3f, %0.3f, %0.3f\n", out4[0], out4[1], out4[2]);
  verify_vect(ans4, out4);

  // testing quaternion to up function
  float q5[4]   = { 0.1013,  0.1578, -0.9591,  0.2119};
  float ans5[3] = {-0.1274, -0.4384, -0.8897};
  float out5[3] = { 0.0000,  0.0000,  0.0000};
  IMU_math_quatToUp(q5, out5);
  printf("%0.3f, %0.3f, %0.3f\n", out5[0], out5[1], out5[2]);
  verify_vect(ans5, out5);

  // testing quaternion to forward function
  float q6[4]   = { 0.4829, -0.3733,  0.6402, -0.4644};
  float ans6[3] = {-0.2548, -0.9285, -0.2701};
  float out6[3] = { 0.0000,  0.0000,  0.0000};
  IMU_math_quatToFrwd(q6, out6);
  printf("%0.3f, %0.3f, %0.3f\n", out6[0], out6[1], out6[2]);
  verify_vect(ans6, out6);

  // testing up to quaternion function
  float u7[3]   = { 0.3594,  0.3102, -0.6748};
  float ans7[4] = { 0.3018, -0.6229,  0.7217,  0.0000};
  float out7[4] = { 0.0000,  0.0000,  0.0000,  0.0000};
  IMU_math_upToQuat(u7, out7);
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", out7[0], out7[1], out7[2], out7[3]);
  verify_quat(ans7, out7);

  // testing two vector to quaternion function
  float u8[3]   = { 0.6294,  0.8116, -0.7460};
  float v8[3]   = { 0.8268,  0.2647, -0.8049};
  float ans8[4] = { 0.9718, -0.1560, -0.0377, -0.1727};
  float out8[4] = { 0.0000,  0.0000,  0.0000,  0.0000};
  IMU_math_vectToQuat(u8, v8, out8);
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", out8[0], out8[1], out8[2], out8[3]);
  verify_quat(ans8, out8);

  // testing up-forward to quaternion function
  float u9[3]   = {   -6.6,  -244.1,   -73.4};
  float f9[3]   = {  250.5,   -19.8,    43.2};
  float ans9[4] = { 0.5949,  0.7983, -0.0821,  0.0449};
  float out9[4] = { 0.0000,  0.0000,  0.0000,  0.0000};
  IMU_math_upFrwdToQuat(u9, f9, out9);
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", out9[0], out9[1], out9[2], out9[3]);
  verify_quat(ans9, out9);

  // testing up-forward to quaternion function
  float u10[3]   = {  147.4,    91.3,  -187.0};
  float f10[3]   = {  197.5,    10.8,   161.0};
  float ans10[4] = {-0.1647,  0.9274,  0.0806,  0.3260};
  float out10[4] = { 0.0000,  0.0000,  0.0000,  0.0000};
  IMU_math_upFrwdToQuat(u10, f10, out10);
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", out10[0], out10[1], out10[2], out10[3]);
  verify_quat(ans10, out10);

  // testing up-forward to quaternion function
  float u11[3]   = {    2.5,   220.5,  -128.1};
  float f11[3]   = { -245.1,    37.3,    59.5};
  float ans11[4] = {-0.0651,  0.1228,  0.8580,  0.4945};
  float out11[4] = { 0.0000,  0.0000,  0.0000,  0.0000};
  IMU_math_upFrwdToQuat(u11, f11, out11);
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", out11[0], out11[1], out11[2], out11[3]);
  verify_quat(ans11, out11);

  // testing up-forward to quaternion function
  float u12[3]   = {   35.6,   250.4,    32.7};
  float f12[3]   = {  -24.9,   -29.3,   252.1};
  float ans12[4] = {-0.4690,  0.4808,  0.4525,  0.5866};
  float out12[4] = { 0.0000,  0.0000,  0.0000,  0.0000};
  IMU_math_upFrwdToQuat(u12, f12, out12);
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", out12[0], out12[1], out12[2], out12[3]);
  verify_quat(ans12, out12);

  // exit program
  printf("pass: test_quat_math\n\n");
  return 0;
}

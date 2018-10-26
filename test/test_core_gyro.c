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
#include <unistd.h>
#include <math.h>
#include "IMU_engn.h"
#include "test_utils.h"

// define globals
uint16_t    id          = 0;
float       curTime     = 0;

// define internal function
static void test_datum   (float gyro[3], float dt, int num_iter, float ref[4]);
static void verify_estm  (float ref[4]);


/******************************************************************************
* main function
******************************************************************************/

int main(void)
{
  // define local constants
  float              dt        = 0.1;
  int                num_iter  = 60; 

  // define local variable
  int                status;

  // start datum test
  printf("starting test_core_gyro...\n");

  // initialize imu engine
  status = IMU_engn_init(IMU_engn_core_only, &id);
  check_status(status, "IMU_engn_init failure");
  status = IMU_engn_load(id, "../config/test_core.json", IMU_engn_core);
  check_status(status, "IMU_engn_load failure");
  status = IMU_engn_start();
  check_status(status, "IMU_engn_start failure");

  /****************************************************************************
  * main test
  ****************************************************************************/

  // verify quaternion initialization
  float gyro1[3]     = {     67,       0,       0};
  float ref1[4]      = { 0.7108,  0.7108,  0.0000,  0.0000};
  test_datum(gyro1, dt, num_iter, ref1);
  
  // testing accelerometer zeroing
  float gyro2[3]     = {      0,      67,       0};
  float ref2[4]      = { 0.7108,  0.0000,  0.7108,  0.0000};
  test_datum(gyro2, dt, num_iter, ref2);
  
  // verify non-zeroing operation 
  float gyro3[3]     = {      0,       0,      67};
  float ref3[4]      = { 0.7108,  0.0000,  0.0000,  0.7108};
  test_datum(gyro3, dt, num_iter, ref3);
  

  /****************************************************************************
  * exit unit test
  ****************************************************************************/

  // exit program
  printf("pass: test_core_gyro\n\n");
  return 0;
}


/******************************************************************************
* test quaternion and state for a specified datum
******************************************************************************/

void test_datum(
  float                    gyro[3],
  float                    dt,
  int                      num_iter,
  float                    ref[4])
{
  // define local variable
  IMU_datum                datum;
  int                      status;
  int                      i;

  // increment time
  curTime                 += dt;
  datum.type               = IMU_gyro;
  datum.val[0]             = gyro[0];
  datum.val[1]             = gyro[1];
  datum.val[2]             = gyro[2];

  // inject datum
  status = IMU_engn_reset(id);
  check_status(status, "IMU_engn_reset failure");
  for (i=0; i<num_iter; i++) {
    curTime               += dt;
    datum.t                = curTime * 100000;
    status                 = IMU_engn_datum(0, &datum);
    check_status(status, "IMU_engn_datum failure");
    usleep(msg_delay);
  }

  // verify value
  verify_estm(ref);
}


/******************************************************************************
* prints system quaternion and verifies against a reference
******************************************************************************/

void verify_estm(
  float                    ref[4])
{
  // get quaternion estimate
  IMU_engn_estm            estm;
  IMU_engn_getEstm(id, curTime, &estm);
  float *q = estm.q;
  
  // print and verify current quaternion
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", q[0], q[1], q[2], q[3]);
  verify_quat(q, ref);
}

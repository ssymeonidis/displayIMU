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

// define constants
static const int   msg_delay   = 200;
static const float precision   = 0.01;

// define globals
uint16_t           id          = 0;
IMU_TYPE           curTime     = 0;

// define internal function
static void test_datum   (float vec[3], int num_iter, float ref[4], int state);
static void verify_estm  (float ref[4]);


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // define local variable
  IMU_engn_config    *engnConfig;
  int                status;

  // start datum test
  printf("starting test_core_accl...\n");

  // initialize imu engine
  status = IMU_engn_init(IMU_engn_core_only, &id, &engnConfig);
  check_status(status, "IMU_engn_init failure");
  status = IMU_engn_load(id, "../config/test_core.json", IMU_engn_core);
  check_status(status, "IMU_engn_load failure");
  status = IMU_engn_start();
  check_status(status, "IMU_engn_start failure");

  // verify quaternion initialization
  float out1[4]      = { 1.0000,  0.0000,  0.0000,  0.0000};
  verify_estm(out1);
  
  // testing accelerometer zeroing
  float vec1[3]      = {    179,     179,     255};
  float out2[4]      = { 0.9246, -0.2694,  0.2694,  0.0000};
  test_datum(vec1, 1, out2, IMU_CORE_ZEROED_ACCL);
  
  // testing initalization of gyroscope related state values
  float vec2[3]      = {      0,       0,     255};
  test_datum(vec2, 1, out2, IMU_CORE_NORMAL_OP);
  
  // testing initalization of gyroscope related state values
  test_datum(vec2, 100, out1, IMU_CORE_NORMAL_OP);
  
  // exit program
  printf("pass: test_core_accl\n\n");
  return 0;
}


/******************************************************************************
* assess quaternion based on datum
******************************************************************************/

void test_datum(
  float                    vec[3],
  int                      num_iter,
  float                    ref[4],
  int                      state)
{
  // define local variable
  IMU_datum                datum;
  int                      status;
  int                      i;

  // increment time
  curTime                 += 10;
  datum.type               = IMU_accl;
  datum.t                  = curTime;
  datum.val[0]             = vec[0];
  datum.val[1]             = vec[1];
  datum.val[2]             = vec[2];

  for (i=0; i<num_iter; i++) {
    status                 = IMU_engn_datum(0, &datum);
    check_status(status, "IMU_engn_datum failure");
    usleep(msg_delay);
  }

  // verify value
  verify_state(state);
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
  
  // print current quaternion
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", q[0], q[1], q[2], q[3]);

  // verify against reference;
  if (fabs(q[0] - ref[0]) > precision ||
      fabs(q[1] - ref[1]) > precision ||
      fabs(q[2] - ref[2]) > precision ||
      fabs(q[3] - ref[3]) > precision) {
    printf("error: quaternion precision error\n");
    exit(0);
  }
}


/******************************************************************************
* verifies state of last datum 
******************************************************************************/

void verify_state(
  int                      state_in)
{
  // define local variables
  IMU_union_state          unionState;
  int status = IMU_engn_getState(id, IMU_engn_core, &unionState);
  int state  = unionState.stateCore->status;
  
  // print current state
  printf("%d, ", state);

  // verify against reference;
  if (state != state_in) {
    printf("error: state error\n");
    exit(0);
  }
}

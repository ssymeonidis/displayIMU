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
uint16_t           id          = 0;
IMU_TYPE           curTime     = 0;

// define internal function
static void test_datum   (float vec[3], int num_iter, float ref[4], int state);
static void add_accl     (float vec[3], float ref[4], int state);
static void verify_estm  (float ref[4]);
static void verify_state (int   state);


/******************************************************************************
* main function
******************************************************************************/

int main(void)
{
  // define local variable
  int                status;

  // start datum test
  printf("starting test_core_magn...\n");

  // initialize imu engine
  status = IMU_engn_init(IMU_engn_core_only, &id);
  check_status(status, "IMU_engn_init failure");
  status = IMU_engn_load(id, "../config/test_core.json", IMU_engn_core);
  check_status(status, "IMU_engn_load failure");
  status = IMU_engn_start();
  check_status(status, "IMU_engn_start failure");


  /****************************************************************************
  * test #1 - no acclerometer for reset
  ****************************************************************************/

  // verify quaternion initialization
  float out_reset[4] = { 1.0000,  0.0000,  0.0000,  0.0000};
  verify_state(0);
  verify_estm(out_reset);
  
  // testing magnetometer zeroing
  float vec1[3]      = {    233,     104,     -66};
  float out1[4]      = { 0.9763,  0.0000,  0.0000,  0.2165};
  test_datum(vec1, 1, out1, IMU_CORE_ZEROED_SAVE);
  
  // verify non-zeroing operation 
  float vec2[3]      = {    255,       0,       0};
  test_datum(vec2, 1, out1, IMU_CORE_NORMAL_OP);
  
  // testing accelerometer filtering
  float out2[4]      = { 1.0000,  0.0000,  0.0000,  0.0000};
  test_datum(vec2, 100, out2, IMU_CORE_NORMAL_OP);


  /****************************************************************************
  * test #2 - magnetometer then accelerometer datum for reset
  ****************************************************************************/

  // reset core and engine
  status = IMU_engn_reset(id);
  check_status(status, "IMU_engn_reset failure");
  verify_state(0);
  verify_estm(out_reset);

  // inject magnetometer datum
  float vec3[3]      = {    104,     223,     -66};
  float out3[4]      = { 0.8434,  0.0000,  0.0000,  0.5373};
  test_datum(vec3, 1, out3, IMU_CORE_ZEROED_SAVE);

  // testing accelerometer zeroing
  float vec4[3]      = {     87,      30,     238};
  float out4[4]      = { 0.8382,  0.0396,  0.1787,  0.5183};
  add_accl(vec4, out4, IMU_CORE_ZEROED_BOTH);

  // verify non-zeroing operation 
  float vec5[3]      = {      0,    -255,       0};
  test_datum(vec5, 1, out4, IMU_CORE_NORMAL_OP);
    
  // testing accelerometer filtering
  float out5[4]      = { 0.7253, -0.0070,  0.1569,  0.6703};
  test_datum(vec5, 100, out5, IMU_CORE_NORMAL_OP);


  /****************************************************************************
  * test #3 - accelerometer then magnetomter datum for reset
  ****************************************************************************/

  // reset core and engine
  status = IMU_engn_reset(id);
  check_status(status, "IMU_engn_reset failure");
  verify_state(0);
  verify_estm(out_reset);

  // testing accelerometer zeroing
  float vec6[3]      = {    -33,     -86,     238};
  float out6[4]      = { 0.9830,  0.1714, -0.0658,  0.0000};
  add_accl(vec6, out6, IMU_CORE_ZEROED_ACCL);
  
  // inject magnetometer datum
  float vec8[3]      = {    104,    -223,     -66};
  float out8[4]      = { 0.8199,  0.1787,  0.0396, -0.5425};
  test_datum(vec8, 1, out8, IMU_CORE_ZEROED_MAGN);

  // verify non-zeroing operation 
  float vec9[3]      = {      0,     255,       0};
  test_datum(vec9, 1, out8, IMU_CORE_NORMAL_OP);
    
  // testing accelerometer filtering
  float out9[4]      = { 0.7445,  0.0294,  0.2336, -0.6247};
  test_datum(vec9, 100, out9, IMU_CORE_NORMAL_OP);


  /****************************************************************************
  * exit unit test
  ****************************************************************************/

  // exit program
  printf("pass: test_core_magn\n\n");
  return 0;
}


/******************************************************************************
* test quaternion and state for a specified datum
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
  datum.type               = IMU_magn;
  datum.t                  = curTime;
  datum.val[0]             = vec[0];
  datum.val[1]             = vec[1];
  datum.val[2]             = vec[2];

  // inject datum
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
* inject magnetometer value (for reset testing)
******************************************************************************/

void add_accl(
  float                    vec[3],
  float                    ref[4],
  int                      state)
{
  // define local variable
  IMU_datum                datum;
  int                      status;

  // increment time
  curTime                 += 10;
  datum.type               = IMU_accl;
  datum.t                  = curTime;
  datum.val[0]             = vec[0];
  datum.val[1]             = vec[1];
  datum.val[2]             = vec[2];

  // inject datum
  status                 = IMU_engn_datum(0, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(msg_delay);

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
  
  // print and verify current quaternion
  printf("%0.3f, %0.3f, %0.3f, %0.3f\n", q[0], q[1], q[2], q[3]);
  verify_quat(q, ref);
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
  check_status(status, "IMU_engn_getState failure");
  int state  = unionState.core->status;
  
  // print current state
  printf("%d, ", state);

  // verify against reference;
  if (state != state_in) {
    printf("error: state error\n");
    exit(0);
  }
}

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
#include "IMU_pnts.h"
#include "test_utils.h"

// define globals
uint16_t    id          = 0;
float       curTime     = 0;

// define internal function
void test_datum(float g[3], float dt, int pnts, float t, float  ref[3]);
void verify_state(int state);
void verify_tStable(uint32_t tCurrent, float tStable);
void verify_sensor(float ref[3]);


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // define local variable
  IMU_union_config     config;
  int                  status;

  // start datum test
  printf("starting test_pnts_gyro...\n");

  // initialize the IMU and its data parser
  status = IMU_engn_init(IMU_engn_calb_pnts, &id);
  check_status(status, "IMU_engn_init failure");
  
  // enable sensor structure
  status = IMU_engn_getConfig(id, IMU_engn_self, &config);
  check_status(status, "IMU_engn_getConfig failure");
  config.engn->isSensorStruct = 1;
  
  // disable core (not under test)
  status = IMU_engn_getConfig(id, IMU_engn_core, &config);
  check_status(status, "IMU_engn_getConfig failure");
  config.core->enable = 0;

  // disable rect (not under test)
  status = IMU_engn_getConfig(id, IMU_engn_rect, &config);
  check_status(status, "IMU_engn_getConfig failure");
  config.rect->enable = 0;

  // read IMU_pnts json config file
  status = IMU_engn_load(id, "../config/test_pnts.json", IMU_engn_pnts);
  check_status(status, "IMU_engn_load failure");

  // start data queue
  status = IMU_engn_start();
  check_status(status, "IMU_engn_start failure");


  /****************************************************************************
  * test #1 - stability test
  ****************************************************************************/

  // verify reset condition
  verify_state(IMU_pnts_enum_reset); printf("\n");

  // verify initialization
  float vec1[3] = {10.0, 20.0, 30.0};
  test_datum(vec1, 0.01, IMU_pnts_enum_move,   0.00, vec1);

  // verify tStable
  test_datum(vec1, 0.05, IMU_pnts_enum_move,   0.05, vec1);
  test_datum(vec1, 0.03, IMU_pnts_enum_move,   0.08, vec1);

  // verify hold state
  test_datum(vec1, 0.05, IMU_pnts_enum_hold,   0.13, vec1);
  test_datum(vec1, 0.05, IMU_pnts_enum_hold,   0.18, vec1);

  // verify stable state
  test_datum(vec1, 0.04, IMU_pnts_enum_stable, 0.22, vec1);

  // verify moving state
  float vec2[3] = {30.0, 30.0, 30.0};
  float out2[3] = {15.0, 22.5, 30.0};
  test_datum(vec2, 0.04, IMU_pnts_enum_move,   0.00, out2);


  /****************************************************************************
  * test #2 - hold test
  ****************************************************************************/

  // reset pnts logic
  IMU_engn_reset(id);
  verify_state(IMU_pnts_enum_reset); printf("\n");

  // verify initialization
  test_datum(vec1, 0.01, IMU_pnts_enum_move,   0.00, vec1);

  // verify tStable
  float vec3[3]  = {20.0, 20.0, 20.0};
  float out3a[3] = {12.5, 20.0, 27.5};
  float out3b[3] = {11.9, 20.0, 28.1};
  test_datum(vec3, 0.04, IMU_pnts_enum_move,   0.04, out3a);
  test_datum(vec1, 0.04, IMU_pnts_enum_move,   0.04, out3b);

  // verify hold state
  float out4[3]  = {11.40, 20.0, 28.60};
  test_datum(vec1, 0.04, IMU_pnts_enum_hold,   0.04, out4);

  // verify moving state
  float out5[3]  = {16.05, 22.50, 28.95};
  test_datum(vec2, 0.04, IMU_pnts_enum_move,   0.00, out5);


  /****************************************************************************
  * test #3 - moving test
  ****************************************************************************/

  // verify tStable
  float vec6[3]  = {20.00, 20.00, 30.00};
  float out6a[3] = {17.04, 21.88, 29.21};
  float out6b[3] = {17.78, 21.41, 29.41};
  test_datum(vec6, 0.04, IMU_pnts_enum_move,   0.04, out6a);
  test_datum(vec6, 0.04, IMU_pnts_enum_move,   0.04, out6b);

  // verify tStable clear
  float vec7[3]  = {10.00, 10.00, 10.00};
  float out7[3]  = {15.84, 18.55, 24.56};
  test_datum(vec7, 0.04, IMU_pnts_enum_move,   0.04, out7);

  // verify tStable
  float vec8[3]  = {15.00, 20.00, 25.00};
  float out8[3]  = {15.63, 18.92, 24.67};
  test_datum(vec8, 0.07, IMU_pnts_enum_move,   0.07, out8);


  /****************************************************************************
  * exit unit test
  ****************************************************************************/

  // exit program
  printf("pass: test_ptns_gyro\n\n");
  return 0;
}


/******************************************************************************
* inject sensor datum and verify state
******************************************************************************/

void test_datum(
  float                    gyro[3],
  float                    dt,
  int                      state,
  float                    tStable,
  float                    ref[3])
{
  // define local variable
  IMU_datum                datum;
  int                      status;

  // increment time
  curTime                 += dt;
  datum.type               = IMU_gyro;
  datum.t                  = curTime * 100000;
  datum.val[0]             = gyro[0];
  datum.val[1]             = gyro[1];
  datum.val[2]             = gyro[2];
  status                   = IMU_engn_datum(0, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(msg_delay);

  // verify results
  verify_state(state);
  verify_tStable(datum.t, tStable);
  verify_sensor(ref);
}


/******************************************************************************
* verify pnts state
******************************************************************************/

void verify_state(
  int                      ref)
{
  // define local variable
  IMU_union_state          state;
  int                      status;

  // verify against reference
  status                   = IMU_engn_getState(id, IMU_engn_self, &state);
  check_status(status, "IMU_engn_getState failure");
  printf("%d, ", state.engn->pnts);
  if (state.engn->pnts != ref) {
    printf("error: pnts state failure\n");
    exit(0);
  }
}


/******************************************************************************
* verify tStable
******************************************************************************/

void verify_tStable(
  uint32_t                 tCurrent,
  float                    tStable)
{
  // define local variable
  IMU_union_state          state;
  int                      status;

  // verify against reference
  status                   = IMU_engn_getState(id, IMU_engn_pnts, &state);
  check_status(status, "IMU_engn_getState failure");
  float delta = (float)(tCurrent - state.pnts->tStable) / 100000.0;
  printf("%0.3f, ", delta);
  if (abs(delta - tStable) > precision) {
    printf("error: pnts tStable failure\n");
    exit(0);
  } 
}


/******************************************************************************
* verify sensor
******************************************************************************/

void verify_sensor(
  float                    ref[3])
{
  // define local variable
  IMU_engn_sensor          *sensor;
  int                      status;

  // verify against reference
  status = IMU_engn_getSensor(id, &sensor);
  check_status(status, "IMU_engn_getSensor failure");
  printf("%0.2f, %0.2f, %0.2f\n", 
    sensor->gFlt[0],  sensor->gFlt[1],  sensor->gFlt[2]);
  verify_vect(ref, sensor->gFlt);
}



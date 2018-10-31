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
int         fncVal      = 0;

// define internal function
void test_fnc(uint16_t id, uint16_t cnt, IMU_pnts_entry*, void*);
void test_datum(float g[3], float dt, int state, float t, float  ref[3]);
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
  IMU_union_state      state;
  int                  status;

  // start datum test
  printf("starting test_pnts_fnc...\n");

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

  // set callback function
  status = IMU_engn_setPntsFnc(id, &test_fnc, &fncVal);
  check_status(status, "IMU_engn_setPntsFnc failure");

  // start data queue
  status = IMU_engn_start();
  check_status(status, "IMU_engn_start failure");


  /****************************************************************************
  * test #1 - stability test
  ****************************************************************************/

  // provide non-zero pnts count to allow callback
  status = IMU_engn_getState(id, IMU_engn_pnts, &state);
  check_status(status, "IMU_engn_getState failure");
  state.pnts->numPnts = 1;

  // verify reset condition
  verify_state(IMU_pnts_enum_reset); printf("\n");

  // verify initialization
  float vec1[3] = {10.0, 20.0, 30.0};
  test_datum(vec1, 0.01, IMU_pnts_enum_move,   0.00, vec1);

  // verify stable state
  test_datum(vec1, 0.25, IMU_pnts_enum_stable, 0.25, vec1);

  // verify moving state
  float vec2[3] = {30.0, 30.0, 30.0};
  float out2[3] = {15.0, 22.5, 30.0};
  test_datum(vec2, 0.04, IMU_pnts_enum_move,   0.00, out2);


  /****************************************************************************
  * exit unit test
  ****************************************************************************/

  // exit program
  printf("pass: test_pnts_fnc\n\n");
  return 0;
}


/******************************************************************************
* callback funtion
******************************************************************************/

void test_fnc(
  uint16_t                 id, 
  uint16_t                 count,
  IMU_pnts_entry           *entry,
  void                     *pntr)
{
  printf("here\n");
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
  if (fabs(delta - tStable) > 0.00001) {
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

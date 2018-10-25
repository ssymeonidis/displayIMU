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

// define constants
static const int   msg_delay   = 200;
static const float precision   = 0.001;

// define internal function
static void print_gyro(IMU_TYPE time, IMU_engn_sensor*, IMU_pnts_state*);
static void verify_vector(float *val, float ref[3]);


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // define local variable
  IMU_datum          datum;
  IMU_engn_config    *engnConfig;
  IMU_union_config   unionConfig;
  IMU_union_state    unionState;
  IMU_core_config    *coreConfig;
  IMU_core_state     *coreState;
  uint16_t           id;
  int                status;

  // start datum test
  printf("starting test_core_accl...\n");

  // initialize the IMU and its data parser
  status = IMU_engn_init(IMU_engn_core_only, &id, &engnConfig);
  if (status < 0) {
    printf("error: IMU_engn_init failure #%d\n", status);
    exit(0);
  }
  
  // get pointer to core config structure
  status = IMU_engn_getConfig(id, IMU_engn_core, &unionConfig);
  if (status < 0) {
    printf("error: IMU_engn_getConfig failure #%d\n", status);
    exit(0);
  }
  coreConfig = unionConfig.configCore;

  // get pointer to core state structure
  status = IMU_engn_getState(id, IMU_engn_core, &unionState);
  if (status < 0) {
    printf("error: IMU_engn_getState failure #%d\n", status);
    exit(0);
  }
  coreState = unionState.stateCore;

  // read IMU_pnts json config file
  status = IMU_engn_load(id, "../config/test_core.json", IMU_engn_core);
  if (status < 0) {
    printf("error: IMU_engn_load failure #%d\n", status);
    exit(0);
  }

  // start data queue
  status = IMU_engn_start();
  if (status < 0) {
    printf("error: IMU_engn_start failure #%d\n", status);
    exit(0);
  }
  
  // testing initalization of gyroscope related state values
  /*datum.type    = IMU_gyro;
  datum.t       = 10;
  datum.val[0]  = 10;
  datum.val[1]  = 20;
  datum.val[2]  = 30;
  status = IMU_engn_datum(id, &datum);
  if (status < 0) {
    printf("error: IMU_engn_datum failure #%d\n", status);
    exit(0);
  }*/
  
  // exit program
  printf("pass: test_ptns_gyro\n\n");
  return 0;
}


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

void print_gyro(
  IMU_TYPE                 time,
  IMU_engn_sensor          *sensor, 
  IMU_pnts_state           *state)
{
  // get pointer to points entry
  IMU_pnts_entry *entry = state->current;
  printf("%d, %d, %d, %d, %d, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, ", 
    state->curPnts, state->state, time-state->tStable, 
    entry->tStart, entry->tEnd,
    state->gMean[0],  state->gMean[1],  state->gMean[2],
    entry->gAccum[0], entry->gAccum[1], entry->gAccum[2]);
  printf("%0.1f, %0.1f, %0.1f\n", 
    sensor->gFlt[0],  sensor->gFlt[1],  sensor->gFlt[2]);
}


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

void verify_vector(
  float                    *val,
  float                    ref[3])
{
  if (fabs(val[0] - ref[0]) > precision ||
      fabs(val[1] - ref[1]) > precision ||
      fabs(val[2] - ref[2]) > precision) {
    printf("error: gMean precision failure\n");
    exit(0);
  }
}

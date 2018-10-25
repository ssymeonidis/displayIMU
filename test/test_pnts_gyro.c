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
  IMU_engn_sensor    *sensor;
  IMU_union_config   config;
  IMU_union_state    state;
  IMU_pnts_state     *pnts;
  uint16_t           id;
  int                status;

  // start datum test
  printf("starting test_pnts_gyro...\n");

  // initialize the IMU and its data parser
  status = IMU_engn_init(IMU_engn_calb_pnts, &id);
  check_status(status, "IMU_engn_init failure");
  
  // get pointer to sensor structure
  status = IMU_engn_getConfig(id, IMU_engn_self, &config);
  check_status(status, "IMU_engn_getConfig failure");
  config.engn->isSensorStruct = 1;
  status = IMU_engn_getSensor(id, &sensor);
  check_status(status, "IMU_engn_getSensor failure");
  
  // get pointer to pnts state structure
  status = IMU_engn_getState(id, IMU_engn_pnts, &state);
  check_status(status, "IMU_engn_getState failure");
  pnts = state.pnts;
  if (pnts == NULL) {
    printf("error: IMU_engn_getState return NULL pointer");
    exit(0);
  }
  
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
  
  // start points collections
  status = IMU_engn_calbStart(id, IMU_calb_NA); 
  check_status(status, "IMU_engn_calbStart failure");
  
  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 10;
  datum.val[0]  = 10;
  datum.val[1]  = 20;
  datum.val[2]  = 30;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  float zero[3] = {0,  0,  0};
  float val1[3] = {10, 20, 30};
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  verify_vector(pnts->gMean,  val1);
  verify_vector(sensor->gFlt, zero);

  // testing averaging of gyroscope data
  datum.type    = IMU_gyro;
  datum.t       = 15;
  datum.val[0]  = 5;
  datum.val[1]  = 25;
  datum.val[2]  = 35;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  float val2[3] = {9.5, 20.5, 30.5};
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  verify_vector(pnts->gMean,  val2);
  verify_vector(sensor->gFlt, zero);

  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 115;
  datum.val[0]  = 10;
  datum.val[1]  = 20;
  datum.val[2]  = 30;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  float val4[3] = {10, 20, 30};
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  verify_vector(sensor->gFlt, val4);

  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 165;
  datum.val[0]  = 15;
  datum.val[1]  = 15;
  datum.val[2]  = 25;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  float val5[3] = {10.5, 19.5, 29.5};
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  verify_vector(sensor->gFlt, val5);

  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 215;
  datum.val[0]  = 5;
  datum.val[1]  = 25;
  datum.val[2]  = 35;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  float val6[3] = {9.95, 20.05, 30.05};
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  verify_vector(sensor->gFlt, val6);

  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 250;
  datum.val[0]  = 40;
  datum.val[1]  = 40;
  datum.val[2]  = 40;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  if (pnts->curPnts != 1) {
    printf("error: IMU point not captured #%d\n", status);
    exit(0);
  }
  verify_vector(sensor->gFlt, zero);

  // testing initalization of gyroscope relLated state values
  datum.type    = IMU_gyro;
  datum.t       = 260;
  datum.val[0]  = 10;
  datum.val[1]  = 20;
  datum.val[2]  = 30;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  if (pnts->state != IMU_pnts_enum_unstable) {
    printf("error: IMU state not unstable\n");
    exit(0);
  }

  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 380;
  datum.val[0]  = 10;
  datum.val[1]  = 20;
  datum.val[2]  = 30;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  if (pnts->state != IMU_pnts_enum_stable) {
    printf("error: IMU state not stable\n");
    exit(0);
  }
  
  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 400;
  datum.val[0]  = 40;
  datum.val[1]  = 40;
  datum.val[2]  = 40;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  if (pnts->curPnts != 1) {
    printf("error: min hold time violated\n");
    exit(0);
  }
  verify_vector(sensor->gFlt, zero);
  if (pnts->state == IMU_pnts_enum_stable) {
    printf("error: IMU state should not be stable\n");
    exit(0);
  }

  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 420;
  datum.val[0]  = 10;
  datum.val[1]  = 20;
  datum.val[2]  = 30;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  
  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 480;
  datum.val[0]  = 10;
  datum.val[1]  = 20;
  datum.val[2]  = 30;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  if (datum.t-pnts->tStable < 60) {
    printf("error: IMU time stability error #%d\n", status);
    exit(0);
  }
  
  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 500;
  datum.val[0]  = 40;
  datum.val[1]  = 40;
  datum.val[2]  = 40;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  if (pnts->state == IMU_pnts_enum_stable) {
    printf("error: IMU state should not be stable\n");
    exit(0);
  }
   
  // testing initalization of gyroscope related state values
  datum.type    = IMU_gyro;
  datum.t       = 560;
  datum.val[0]  = 10;
  datum.val[1]  = 20;
  datum.val[2]  = 30;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify IMU_pnts state structure
  usleep(msg_delay);
  print_gyro(datum.t, sensor, pnts);
  if (pnts->state == IMU_pnts_enum_stable) {
    printf("error: IMU state should not be stable\n");
    exit(0);
  }
   
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

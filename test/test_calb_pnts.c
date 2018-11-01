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
void add_datum(float val[3], float off[3], IMU_sensor);


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // define local variable
  IMU_union_config     config;
  int                  status;

  // start datum test
  printf("starting test_calb_pnts...\n");

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

  // read IMU_pnts and IMU_calb json config file
  status = IMU_engn_load(id, "../config/test_pnts.json", IMU_engn_pnts);
  check_status(status, "IMU_engn_load failure");
  status = IMU_engn_load(id, "../config/test_calb.json", IMU_engn_calb);
  check_status(status, "IMU_engn_load failure");

  // start data queue
  status = IMU_engn_start();
  check_status(status, "IMU_engn_start failure");

  // change alpha to allow easier point triggering
  status = IMU_engn_getConfig(id, IMU_engn_pnts, &config);
  check_status(status, "IMU_engn_getConfig failure");
  config.pnts->gAlpha = 1.0;
  config.pnts->aAlpha = 1.0;
  config.pnts->mAlpha = 1.0;

  // get handle to config status
  status = IMU_engn_getConfig(id, IMU_engn_rect, &config);
  check_status(status, "IMU_engn_getConfig failure");
  IMU_rect_config *rect   = config.rect;


  /****************************************************************************
  * test one-point gyroscope NUC
  ****************************************************************************/

  // initiate one-point nuc
  float off1[3]   = { 50.0,  50.0,  50.0};
  float vec1[3]   = { 10.0,  20.0,  30.0};
  float ref1[3]   = {-10.0, -20.0, -30.0};
  status = IMU_engn_calbStart(id, IMU_calb_1pnt_gyro);
  add_datum(vec1, off1, IMU_gyro);
  print_vect(rect->gBias);
  verify_vect(rect->gBias, ref1);


  /****************************************************************************
  * test four-point magnetometer NUC
  ****************************************************************************/

  // initiate one-point nuc
  float off2[3]    = { 99.0,  99.0, 99.0};
  float vec2a[3]   = { 37.0,  -2.0,  0.0};
  float vec2b[3]   = {-23.0,  -2.0,  0.0};
  float vec2c[3]   = {  7.0,  28.0,  0.0};
  float vec2d[3]   = {  7.0, -32.0,  0.0};
  float ref2[3]    = { -7.0,   2.0,  0.0};
  status = IMU_engn_calbStart(id, IMU_calb_4pnt_magn);
  add_datum(vec2a, off2, IMU_magn);
  add_datum(vec2b, off2, IMU_magn);
  add_datum(vec2c, off2, IMU_magn);
  add_datum(vec2d, off2, IMU_magn);
  print_vect(rect->mBias);
  verify_vect(rect->mBias, ref2);


  /****************************************************************************
  * exit unit test
  ****************************************************************************/

  // exit program
  printf("pass: test_calb_pnts\n\n");
  return 0;
}


/******************************************************************************
* inject stable point (include break_stable condition)
******************************************************************************/

void add_datum(
  float                    val[3],
  float                    off[3],
  IMU_sensor               type)
{
  // define local variable
  IMU_datum                datum;
  int                      status;

  // ceate datum structure
  datum.type               = type;
  datum.val[0]             = val[0];
  datum.val[1]             = val[1];
  datum.val[2]             = val[2];

  // inject first datum (reset reference)
  curTime                 += 0.01;
  datum.t                  = curTime * 100000;
  status                   = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(msg_delay);

  // inject next datum (duration greater than tStable)
  curTime                 += 0.25;
  datum.t                  = curTime * 100000;
  status                   = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(msg_delay);

  // inject last datum (creates break_motion trigger)
  curTime                 += 0.01;
  datum.t                  = curTime * 100000;
  datum.val[0]             = off[0];
  datum.val[1]             = off[1];
  datum.val[2]             = off[2];
  status                   = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(2*msg_delay);
}

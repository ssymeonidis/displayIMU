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
static void test_datum_mag (float vec[3], float FOM);
static void test_datum_dot (float vec[3], float FOM);
static void add_magn       (float vec[3]);
static void add_accl       (float vec[3]);


/******************************************************************************
* main function
******************************************************************************/

int main(void)
{
  // define local variable
  IMU_union_config     config;
  int                  status;

  // start datum test
  printf("starting test_fom_magn...\n");

  // initialize imu engine
  status = IMU_engn_init(IMU_engn_core_only, &id);
  check_status(status, "IMU_engn_init failure");
  status = IMU_engn_load(id, "../config/test_core.json", IMU_engn_core);
  check_status(status, "IMU_engn_load failure");
  status = IMU_engn_start();
  check_status(status, "IMU_engn_start failure");

  // enable sensor structure
  status = IMU_engn_getConfig(id, IMU_engn_self, &config);
  check_status(status, "IMU_engn_getConfig failure");
  config.engn->isFOM          = 1;
  config.engn->isSensorStruct = 1;
  
  // configure FOM core state
  status = IMU_engn_getConfig(id, IMU_engn_core, &config);
  check_status(status, "IMU_engn_getConfig failure");
  config.core->isFOM          = 1;
  config.core->mMag           = 255.0;
  config.core->mMagThresh     = 128.0;
  config.core->mDot           = 0.749;
  config.core->mDotThresh     = 0.150;


  /****************************************************************************
  * testing magnitude error
  ****************************************************************************/

  // testing full weight condition
  float vec1[3]      = {    127,     127,     181};
  test_datum_mag(vec1, 0.0);
  test_datum_mag(vec1, 1.0);

  // testing no weight condition
  float vec2[3]      = {     64,      64,      64};
  test_datum_mag(vec2, 0.0);
  float vec3[3]      = {    250,     200,     300};
  test_datum_mag(vec3, 0.0);

  // testing mid weight condition
  float vec4[3]      = {    116,      97,     281};
  float vec5[3]      = {    146,      93,      81};
  test_datum_mag(vec4, 0.5);
  test_datum_mag(vec5, 0.5);
  
  // testing mid weight condition
  float vec6[3]      = {    186,     123,     181};
  float vec7[3]      = {     66,     120,      81};
  test_datum_mag(vec6, 0.75);
  test_datum_mag(vec7, 0.25);


  /****************************************************************************
  * testing angle error
  ****************************************************************************/

  // testing full weight condition
  float accl[3]      = {     66,      40,     164};
  float vec8[3]      = {    194,      78,      92};
  add_accl(accl);
  test_datum_dot(vec8, 1.00);

  // testing mid weight condition
  float vec9[3]      = {    172,      96,     108};
  test_datum_dot(vec9, 0.62);


  /****************************************************************************
  * exit unit test
  ****************************************************************************/

  // exit program
  printf("pass: test_fom_magn\n\n");
  return 0;
}


/******************************************************************************
* inject datum and verify figure of merit
******************************************************************************/

void test_datum_mag(
  float                    vec[3],
  float                    FOM)
{
  // define local variable
  IMU_engn_sensor          *sensor;
  int                      status;

  // add datum
  add_magn(vec);
  
  // extract figure of merit
  status = IMU_engn_getSensor(id, &sensor);
  check_status(status, "IMU_engn_getSensor failure");
  float magFOM             = sensor->mFOM.magFOM;

  // update display
  printf("%0.2f, %0.2f, %0.2f, %0.2f\n", magFOM, vec[0], vec[1], vec[2]);

  // verify value
  if (fabs(magFOM - FOM) > 0.01) {
    printf("error: magFOM failure\n");
    exit(0);
  }
}


/******************************************************************************
* inject datum and verify figure of merit
******************************************************************************/

void test_datum_dot(
  float                    vec[3],
  float                    FOM)
{
  // define local variable
  IMU_engn_sensor          *sensor;
  int                      status;

  // add datum
  add_magn(vec);
    
  // extract figure of merit
  status = IMU_engn_getSensor(id, &sensor);
  check_status(status, "IMU_engn_getSensor failure");
  float dotFOM             = sensor->mFOM.dotFOM;

  // update display
  printf("%0.2f, %0.2f, %0.2f, %0.2f\n", dotFOM, vec[0], vec[1], vec[2]);

  // verify value
  if (fabs(dotFOM - FOM) > 0.01) {
    printf("error: magFOM failure\n");
    exit(0);
  }
}


/******************************************************************************
* inject magnetometer data
******************************************************************************/

void add_magn(
  float                    vec[3])
{
  // define local variable
  IMU_datum                datum;
  int                      status;

  // populate and add datum
  curTime                 += 10;
  datum.type               = IMU_magn;
  datum.t                  = curTime;
  datum.val[0]             = vec[0];
  datum.val[1]             = vec[1];
  datum.val[2]             = vec[2];
  status                   = IMU_engn_datum(0, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(msg_delay);
}


/******************************************************************************
* inject acclerometer data
******************************************************************************/

void add_accl(
  float                    vec[3])
{
  // define local variable
  IMU_datum                datum;
  int                      status;

  // populate and add datum
  curTime                 += 10;
  datum.type               = IMU_accl;
  datum.t                  = curTime;
  datum.val[0]             = vec[0];
  datum.val[1]             = vec[1];
  datum.val[2]             = vec[2];
  status                   = IMU_engn_datum(0, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(msg_delay);
}

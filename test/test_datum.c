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
#include "IMU_engn.h"
#include "test_utils.h"

// define local variables
const int precision_int  = 2;

// define internal functions
void print_line (uint32_t t, IMU_TYPE val1[3], IMU_TYPE val2[3]);
void verify_uint32   (uint32_t val1,    uint32_t val2);
void verify_IMU_TYPE (IMU_TYPE val1[3], IMU_TYPE val2[3]);


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // define local variable
  IMU_datum          datum;
  IMU_union_config   config;
  IMU_engn_sensor    *sensor;
  uint16_t           id;
  int                status;

  // start datum test
  printf("starting test_datum...\n");

  // initialize the IMU and its data parser
  status = IMU_engn_init(IMU_engn_rect_core, &id);
  check_status(status, "IMU_engn_init failure");
  
  // get pointer to sensor structure
  status = IMU_engn_getConfig(id, IMU_engn_self, &config);
  check_status(status, "IMU_engn_getConfig failure");
  config.engn->isSensorStruct = 1;
  status = IMU_engn_getSensor(id, &sensor);
  check_status(status, "IMU_engn_getSensor failure");
    
  // disable core (not under test)
  status = IMU_engn_getConfig(id, IMU_engn_core, &config);
  check_status(status, "IMU_engn_getConfig failure");
  config.core->enable = 0;

  // read IMU_rect json config file
  status = IMU_engn_load(id, "../config/test_datum.json", IMU_engn_rect);
  check_status(status, "IMU_engn_load failure");

  // start data queue
  status = IMU_engn_start();
  check_status(status, "IMU_engn_start failure");

  
  /****************************************************************************
  * test #1 - inject gyroscope value
  ****************************************************************************/

  // inject gyroscope input
  datum.type           = IMU_gyro;
  datum.t              = 10;
  datum.val[0]         = 100;
  datum.val[1]         = 200;
  datum.val[2]         = 300;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(msg_delay);
  
  // verify sensor structure
  IMU_TYPE refRaw1[3]  = {100, 200, 300};
  IMU_TYPE refCor1[3]  = {220, 330, 110};
  print_line(sensor->time, sensor->gRaw, sensor->gCor);
  verify_uint32(sensor->time, datum.t);
  verify_IMU_TYPE(sensor->gRaw, refRaw1);
  verify_IMU_TYPE(sensor->gCor, refCor1);
  

  /****************************************************************************
  * test #2 - inject accelerometer value
  ****************************************************************************/

  // inject accelerometer input
  datum.type           = IMU_accl;
  datum.t              = 20;
  datum.val[0]         = 400;
  datum.val[1]         = 500;
  datum.val[2]         = 600;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(msg_delay);
  
  // verify sensor structure
  IMU_TYPE refRaw2[3]  = {400, 500, 600};
  IMU_TYPE refCor2[3]  = {660, 440, 550};
  print_line(sensor->time, sensor->gRaw, sensor->gCor);
  verify_uint32(sensor->time, datum.t);
  verify_IMU_TYPE(sensor->aRaw, refRaw2);
  verify_IMU_TYPE(sensor->aCor, refCor2);


  /****************************************************************************
  * test #3 - inject magnetometer value
  ****************************************************************************/

  // inject magnetomter input
  datum.type           = IMU_magn;
  datum.t              = 30;
  datum.val[0]         = 700;
  datum.val[1]         = 800;
  datum.val[2]         = 900;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  usleep(msg_delay);
  
  // verify sensor structure
  IMU_TYPE refRaw3[3]  = {700, 800, 900};
  IMU_TYPE refCor3[3]  = {770, 880, 990};
  print_line(sensor->time, sensor->mRaw, sensor->mCor);
  verify_uint32(sensor->time, datum.t);
  verify_IMU_TYPE(sensor->mRaw, refRaw3);
  verify_IMU_TYPE(sensor->mCor, refCor3);


  /****************************************************************************
  * exit unit test
  ****************************************************************************/

  // exit program
  printf("pass: test_datum\n\n");
  return 0;
}



/******************************************************************************
* prints system quaternion and verifies against a reference
******************************************************************************/

void print_line(
  uint32_t                 t,
  IMU_TYPE                 val1[3],
  IMU_TYPE                 val2[3])
{
  printf("%d, %d, %d, %d, %d, %d, %d\n", t,
    val1[0], val1[1], val1[2],
    val2[0], val2[1], val2[2]);
}


/******************************************************************************
* verifies vector against intended result
******************************************************************************/

void verify_uint32(
  uint32_t             val1,
  uint32_t             val2)
{
  if (fabs(val1 - val2) > precision_int) {
    printf("error: vect results failure\n");
    exit(0);
  }
}


/******************************************************************************
* verifies vector against intended result
******************************************************************************/

void verify_IMU_TYPE(
  IMU_TYPE             val1[3],
  IMU_TYPE             val2[3])
{
  if (fabs(val1[0] - val2[0]) > precision_int ||
      fabs(val1[1] - val2[1]) > precision_int ||
      fabs(val1[2] - val2[2]) > precision_int) {
    exit(0);
  }
}

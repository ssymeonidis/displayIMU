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

// define constants
static const int msg_delay   = 200;


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(void)
{
  // define local variable
  IMU_datum          datum;
  IMU_engn_config    *imuConfig;
  IMU_union_config   configIMU;
  IMU_engn_sensor    *sensor;
  uint16_t           id;
  int                status;

  // start datum test
  printf("starting test_datum...\n");

  // initialize the IMU and its data parser
  status = IMU_engn_init(IMU_engn_rect_core, &id, &imuConfig);
  check_status(status, "IMU_engn_init failure");
  
  // get pointer to sensor structure
  imuConfig->isSensorStruct = 1;
  status = IMU_engn_getSensor(id, &sensor);
  check_status(status, "IMU_engn_getSensor failure");
    
  // disable core (not under test)
  status = IMU_engn_getConfig(id, IMU_engn_core, &configIMU);
  check_status(status, "IMU_engn_getConfig failure");
  configIMU.configCore->enable = 0;

  // read IMU_rect json config file
  status = IMU_engn_load(id, "../config/test_datum.json", IMU_engn_rect);
  check_status(status, "IMU_engn_load failure");

  // start data queue
  status = IMU_engn_start();
  check_status(status, "IMU_engn_start failure");
  
  // inject gyroscope input
  datum.type    = IMU_gyro;
  datum.t       = 10;
  datum.val[0]  = 100;
  datum.val[1]  = 200;
  datum.val[2]  = 300;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify sensor structure
  usleep(msg_delay);
  printf("%d, %d, %d, %d, %d, %d, %d\n", sensor->time,
    sensor->gRaw[0], sensor->gRaw[1], sensor->gRaw[2],
    sensor->gCor[0], sensor->gCor[1], sensor->gCor[2]);
  if (sensor->time    != 10) {
    printf("error: time failure\n");
    exit(0);
  }
  if (sensor->gRaw[0] != 100 || sensor->gRaw[1] != 200 || sensor->gRaw[2] != 300) {
    printf("error: gRaw failure\n");
    exit(0);
  }
  if (sensor->gCor[0] != 220 || sensor->gCor[1] != 330 || sensor->gCor[2] != 110) {
    printf("error: gCor failure\n");
    exit(0);
  }
  
  // inject accelerometer input
  datum.type    = IMU_accl;
  datum.t       = 20;
  datum.val[0]  = 400;
  datum.val[1]  = 500;
  datum.val[2]  = 600;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify sensor structure
  usleep(msg_delay);
  printf("%d, %d, %d, %d, %d, %d, %d\n", sensor->time,
    sensor->aRaw[0], sensor->aRaw[1], sensor->aRaw[2],
    sensor->aCor[0], sensor->aCor[1], sensor->aCor[2]);
  if (sensor->time    != 20) {
    printf("error: time failure\n");
    exit(0);
  }
  if (sensor->aRaw[0] != 400 || sensor->aRaw[1] != 500 || sensor->aRaw[2] != 600) {
    printf("error: aRaw failure\n");
    exit(0);
  }
  if (sensor->aCor[0] != 660 || sensor->aCor[1] != 440 || sensor->aCor[2] != 550) {
    printf("error: aCor failure\n");
    exit(0);
  }

  // inject magnetomter input
  datum.type    = IMU_magn;
  datum.t       = 30;
  datum.val[0]  = 700;
  datum.val[1]  = 800;
  datum.val[2]  = 900;
  status = IMU_engn_datum(id, &datum);
  check_status(status, "IMU_engn_datum failure");
  
  // verify sensor structure
  usleep(msg_delay);
  printf("%d, %d, %d, %d, %d, %d, %d\n", sensor->time,
    sensor->mRaw[0], sensor->mRaw[1], sensor->mRaw[2],
    sensor->mCor[0], sensor->mCor[1], sensor->mCor[2]);
  if (sensor->time    != 30) {
    printf("error: time failure\n");
    exit(0);
  }
  if (sensor->mRaw[0] != 700 || sensor->mRaw[1] != 800 || sensor->mRaw[2] != 900) {
    printf("error: mRaw failure\n");
    exit(0);
  }
  if (sensor->mCor[0] != 770 || sensor->mCor[1] != 880 || sensor->mCor[2] != 990) {
    printf("error: mCor failure\n");
    exit(0);
  }

  // exit program
  printf("pass: test_datum\n\n");
  return 0;
}

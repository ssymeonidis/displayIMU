/*
 * This file is part of quaternion-based displayIMU C/C++/QT code base
 * (https://github.com/ssymeonidis/displayIMU.git)
 * Copyright (c) 2018 Simeon Symeonidis (formerly Sensor Management Real
 * Time (SMRT) Processing Solutions
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
#include "IMU_engn.h"

// define constants
const int msg_delay   = 20;
const int num_samples = 10;


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(
  int                argc, 
  char               *argv[])
{
  // define local variable
  IMU_datum          datum;
  IMU_engn_config    *imuConfig;
  IMU_union_config   configIMU;
  IMU_engn_sensor    *sensor;
  uint16_t           id;
  int                status;
  int                i;

  // initialize the IMU and its data parser
  status = IMU_engn_init(IMU_engn_rect_core, &id, &imuConfig);
  if (status < 0) {
    printf("error: IMU_engn_init failure #%d\n", status);
    exit(0);
  }
  
  // get pointer to sensor structure
  imuConfig->isSensorStruct = 1;
  status = IMU_engn_getSensor(id, &sensor);
  if (status < 0) {
    printf("error: IMU_engn_getSensor failure #%d\n", status);
    exit(0);
  }
    
  // disable core (not under test)
  printf("here #1\n");
  status = IMU_engn_getConfig(id, IMU_engn_core, &configIMU);
  printf("here #2 (%p) - status %d\n", configIMU.configCore, status);
  if (status < 0) {
    printf("error: IMU_engn_getSensor failure #%d\n", status);
    exit(0);
  }
  configIMU.configCore->enable = 0;

  // read IMU_rect json config file
  printf("here #3\n");
  status = IMU_engn_load(id, "test_datum_rect.json", IMU_engn_rect);
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
  
  // inject accelerometer inputs
  datum.type    = IMU_gyro;
  datum.t       = 10;
  datum.val[0]  = 100;
  datum.val[1]  = 200;
  datum.val[2]  = 300;
  status = IMU_engn_datum(id, &datum);
  if (status < 0) {
    printf("error: IMU_engn_start failure #%d\n", status);
    exit(0);
  }
  
  // verify sensor structure
  printf("%f, %f, %f, %f, %f, %f, %f\n", sensor->time,
    sensor->gRaw[0], sensor->gRaw[1], sensor->gRaw[2],
    sensor->gCor[0], sensor->gCor[1], sensor->gCor[2]);
  
  // exit program
  return 0;
}

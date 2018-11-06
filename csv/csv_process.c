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
#include "dataIF.h"
#include "IMU_engn.h"
#include "IMU_util.h"

// define constants
static const int   usleep_delay  = 50;


/******************************************************************************
* main function - simple test of datum queue and correction block
******************************************************************************/

int main(
  int                argc,
  char               *argv[])
{
  // define local variable
  IMU_union_config   config;
  IMU_engn_estm      estm;
  float              *v;
  int                count = 1;
  int                status;

  // initialize the IMU and its data parser
  uint16_t id = dataIF_init(IMU_engn_calb_full);
  
  // verify number of input arguments
  if (argc < 2) {
    printf("error: insuficient number of arguments...\n");
    printf("expecting: csv_process data.csv config_engn.json\n");
    exit(0);
  }

  // read IMU_rect json config file
  if (argc < 3) {
    printf("warning: config file not specified ");
    printf("(using ../config/default_engn.json)\n");
    status = IMU_engn_load(id, "../config/default_engn.json", IMU_engn_self);
  } else {
    status = IMU_engn_load(id, argv[2], IMU_engn_self);
  }
  IMU_util_status(status, "IMU_engn_load failure");

  // get configuration structure
  status = IMU_engn_getConfig(id, IMU_engn_self, &config);
  IMU_util_status(status, "IMU_engn_getConfig failure");

  // start data queue
  status = IMU_engn_start();
  IMU_util_status(status, "IMU_engn_start failure");

  // start file reader
  dataIF_startCSV(argv[1]);


  /****************************************************************************
  * main processing loop
  ****************************************************************************/

  while (1) {

    // add datum from csv file
    status = dataIF_process();
    if (status > 0)
      break;
 
    // get estimate 
    while (1) {
      status = IMU_engn_getEstm(id, 0, &estm);
      printf("status = %d\n", status);
      if (status >= count)
        break;
      usleep(usleep_delay);
    }

    // print estimate
    v = estm.qOrg;
    printf("%0.3f, %0.3f, %0.3f, %0.3f", v[0], v[1], v[2], v[3]);
    if (config.engn->isRef) {
      v = estm.q;
      printf("%0.3f, %0.3f, %0.3f, %0.3f", v[0], v[1], v[2], v[3]);
    }
    if (config.engn->isAng) {
      v = estm.ang;
      printf("%0.1f, %0.1f, %0.1f", v[0], v[1], v[2]);
    }
    if (config.engn->isTran) {
      v = estm.tran;
      printf("%0.1f, %0.1f, %0.1f", v[0], v[1], v[2]);
    }

    // increment count
    count++;
  }
}

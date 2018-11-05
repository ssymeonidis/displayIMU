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
  IMU_datum          datum;
  uint16_t           id;
  int                status;

  // initialize the IMU and its data parser
  IMU_engn_init(IMU_engn_calb_full, &id);
  
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

  // start data queue
  status = IMU_engn_start();
  IMU_util_status(status, "IMU_engn_start failure");

  // start file reader
  dataIF_startCSV(argv[1]);


  /****************************************************************************
  * exit unit test
  ****************************************************************************/

  // exit program
  printf("pass: test_datum\n\n");
  return 0;
}

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
#include <QApplication>
#include <QDesktopWidget>
#include <pthread.h>
#include "dataIF.h"
#include "IMU_util.h"
#include "windowGUI.h"
#include "configGUI.h"


/******************************************************************************
* main function - everything starts here
******************************************************************************/

int main(
  int                argc, 
  char               *argv[])
{
  // define local variables
  int                dataIF_thread_id;
  pthread_t          dataIF_thread;
  int                status;

  // initialize the IMU and its data parser
  uint16_t id = dataIF_init(IMU_engn_calb_full);

  // verify number of variables
  if (argc < 2) {
    printf("warning: config file not specified ");
    printf("(using ../config/default_engn.json)\n");
    status = IMU_engn_load(id, "../config/default_engn.json", IMU_engn_self);
  } else {
    status = IMU_engn_load(id, argv[1], IMU_engn_self);
  }
  IMU_util_status(status, "IMU_engn_load failure");

  // start data queue
  status = IMU_engn_start();
  IMU_util_status(status, "IMU_engn_start failure");

  // create the display
  QApplication app(argc, argv);
  configGUI config("../config/displayIMU.json");
  windowGUI window;
  window.initIMU(&config);
  window.show();

  // launch data parser and IMU interface (seperate threads)
  if (argc < 3) 
    dataIF_startUDP(config.port);
  else
    dataIF_startCSV(argv[2]);
  dataIF_setRealtime();
  dataIF_setRepeat();
  pthread_create(&dataIF_thread, NULL, dataIF_run, &dataIF_thread_id);

  // start the main app
  app.exec();

  // exit program
  // pthread_join(data_thread, NULL);
  return 0;
}

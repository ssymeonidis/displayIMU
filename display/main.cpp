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

#include <QApplication>
#include <QDesktopWidget>
#include <pthread.h>
#include "dataIF.h"
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

  // initialize the IMU and its data parser
  dataIF_init(IMU_engn_rect_core);

  // create the display
  QApplication app(argc, argv);
  configGUI config("../config/displayIMU.json");
  windowGUI window;
  window.initIMU(&config);
  window.show(); 

  // launch data parser and IMU interface (seperate threads)
  if (argc < 2) 
    dataIF_startUDP(config.port);
  else
    dataIF_startCSV(argv[1]);
  pthread_create(&dataIF_thread, NULL, dataIF_run, &dataIF_thread_id);

  // start the main app
  app.exec();

  // exit program
  // pthread_join(data_thread, NULL);
  return 0;
}

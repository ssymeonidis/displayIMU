/*
 * This file is part of quaternion-based displayIMU C++/QT code base
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

#include <QApplication>
#include <QDesktopWidget>
#include <pthread.h>
#include "windowGUI.h"
#include "dataParse.h"

// define internal constants
const int       port_no = 5555;

int main(int argc, char *argv[])
{
  // define internal variables
  int           data_thread_id;
  pthread_t     data_thread;
  char*         config;
  char*         calib;

  // parse command line arguments
  if (argc < 3) {
    config      = NULL;
    calib       = NULL;
  } else {
    config      = NULL;
    calib       = argv[2];
  }

  // create the display
  QApplication  app    (argc,   argv);
  windowGUI     window (config, calib);
  //window.resize(window.sizeHint());
  window.show(); 

  // launch data parser (seperate thread)
  if (argc < 4) 
    data_init_UDP(port_no);
  else
    data_init_CSV(argv[3]);
  pthread_create(&data_thread, NULL, data_run, &data_thread_id);

  // start the main app
  app.exec();

  // exit program
  // pthread_join(data_thread, NULL);
  return 0;
}

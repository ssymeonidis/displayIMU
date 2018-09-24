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
#include <stdio.h>
#include <pthread.h>
#include "window.h"
#include "dataParse.h"

int main(int argc, char *argv[])
{
  // initialize pthread variables
  int       data_thread_id;
  pthread_t data_thread;
  float     config_params[13];
  float     accl_ref_vector[3];
  FILE*     fid;
  int       pass;

  // launch data parser
  if (argc < 2) 
    data_init_UDP(5555);
  else
    data_init_CSV(argv[1]);
  pthread_create(&data_thread, NULL, data_run, &data_thread_id);

  // create the display
  QApplication app(argc, argv);
  Window window(true);
  window.resize(window.sizeHint());
  int desktopArea = QApplication::desktop()->width() *
                    QApplication::desktop()->height();
  int widgetArea = window.width() * window.height();
  if (((float)widgetArea / (float)desktopArea) < 0.75f)
    window.show();
  else
    window.showMaximized();

  // read the configuration params
  if (argc > 2) {
    fid = fopen(argv[2], "r");
    if (fid == NULL)
      printf("cannot open file %s\n", argv[2]);
    else {
      pass = fscanf(fid, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
        &(config_params[0]), &(config_params[1]), &(config_params[2]),
        &(config_params[3]), &(config_params[4]), &(config_params[5]),
        &(config_params[6]), &(config_params[7]), &(config_params[8]),
        &(config_params[9]), &(config_params[10]), &(config_params[11]),
        &(config_params[12]));
      if (pass == 13)
        window.setParams(config_params);
      else
        printf("failed reading file %s\n", argv[2]);
      fclose(fid); 
    }
  }
 
  // read the accelerometer reference vector
  if (argc > 3) {
    fid = fopen(argv[3], "r");
    if (fid == NULL)
      printf("cannot open file %s\n", argv[3]);
    else {
      pass = fscanf(fid, "%f,%f,%f\n", &(accl_ref_vector[0]),
        &(accl_ref_vector[1]), &(accl_ref_vector[2]));
      if (pass == 3) 
        window.setRefAccl(accl_ref_vector);
      else
        printf("failed reading file %s\n", argv[3]);
      fclose(fid); 
    }
  }

  // start the main app
  int exit_code = app.exec();

  // exit program
  // pthread_join(sensor_data_thread, NULL);
  return exit_code;
}

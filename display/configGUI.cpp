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
#include <string.h>
#include "configGUI.h"


/******************************************************************************
* constructor - read config json file
******************************************************************************/

configGUI::configGUI(
  const char          * filename)
{
  // define internal variables
  FILE*     file;
  char*     field;
  char*     args;
  float     val;
  int       status;

  // open json file containing config struct
  file = fopen(filename, "r");
  if (file == NULL)
    return;

  // main loop that parse json file line by line
  while (1) {

    // read line and parse field/args
    status = getLine(file, &field, &args);
    if (status > 1 || status < 0)
      break;

    // extract arguments for the specified field
    if      (strcmp(field, "port") == 0)
      sscanf(args, "%d", &port);
    else if (strcmp(field, "gyro") == 0)
      sscanf(args, "%f", &gyro);
    else if (strcmp(field, "accl") == 0)
      sscanf(args, "%f", &accl);
    else if (strcmp(field, "magn") == 0)
      sscanf(args, "%f", &magn);
    else if (strcmp(field, "IMU")  == 0)
      sscanf(args, "%f", &imu);
  }

  // exit function
  fclose(file);
}


/******************************************************************************
* utility function - gets a line and seperates field from arguments
******************************************************************************/

int configGUI::getLine(
  FILE                 *file, 
  char                 **field, 
  char                 **args)
{
  static const int line_size = 128;
  static char line[line_size];
  static char temp[line_size];
  char *status; 
  status  = fgets(line, line_size, file);
  if (status == NULL)
    return -1;
  *field = strtok(line, ":"); 
  sscanf(*field, "%s", temp);
  if (strcmp(temp, "{") == 0)
    return 1;
  if (strcmp(temp, "}") == 0)
    return 2;
  if (temp == NULL)
    return -1;
  *args = strtok(NULL, "\n");
  strtok(*field, "\"");
  *field = strtok(NULL, "\""); 
  return 0;
} 

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
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <errno.h>
#include "dataIF.h"

// define the data interface structures
dataIF_config            config;
dataIF_state             state;

// define internal/external variables
int                      dataIF_socket;
FILE*                    dataIF_file;

// define internal/utility functions
void dataIF_error(const char *msg);
void dataIF_lineUPD(char* line, int line_size);
int  dataIF_lineCSV(char* line, int line_size);


/******************************************************************************
* constructor for dataParse functions
******************************************************************************/

uint16_t dataIF_init(IMU_engn_type type)
{
  // initialize internal structures
  state.isFirstFrame     = 1;
  state.isExit           = 1;
  config.isRealtime      = 0;
  config.isRepeat        = 0;
  
  // intialize the IMU engine
  int status = IMU_engn_init(type, &state.imuID);
  if (status < 0) {
    printf("initialization error #%d\n", status);
    exit(0);
  }

  // exit function
  return state.imuID;
}


/******************************************************************************
* creates UDP server
******************************************************************************/

void dataIF_startUDP(
  int                    portno)
{
  // define internal variables
  struct sockaddr_in     addr;
  int                    status;
 
  // init IMU and data_stream state
  state.isCSV = 0;

  // open socket
  dataIF_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (dataIF_socket < 0) 
    dataIF_error("ERROR opening socket");

  // bind port 
  bzero((char *)&addr, sizeof(addr));
  addr.sin_family        = AF_INET;
  addr.sin_addr.s_addr   = INADDR_ANY;
  addr.sin_port          = htons(portno);
  status = bind(dataIF_socket, (struct sockaddr *)&addr, sizeof(addr));
  if (status < 0) 
    dataIF_error("ERROR on binding");
}


/******************************************************************************
* opens CSV file
******************************************************************************/

void dataIF_startCSV(
  const char             *filename)
{
  // init IMU and data_stream state
  state.isCSV = 1;

  // fopen file  
  dataIF_file = fopen(filename, "r");
  if (!dataIF_file)
    dataIF_error("ERROR opening csv file"); 
}


/******************************************************************************
* changes to real-time configuation
******************************************************************************/

void dataIF_setRealtime()
{
  config.isRealtime = 1;
}


/******************************************************************************
* changes to real-time configuation
******************************************************************************/

void dataIF_setRepeat()
{
  config.isRepeat = 1;
}


/******************************************************************************
* main function for receiving/parsing data and updating IMU
******************************************************************************/

int dataIF_process()
{
  // define the variables
  int                    status = 0;
  const int              line_size = 256;
  char                   line[line_size];
  int                    datum_type; 

  // extract data line
  if (!state.isCSV)
    dataIF_lineUPD(line, line_size);
  else 
    status = dataIF_lineCSV(line, line_size);
  if (status > 0) {
    usleep(250);
    IMU_engn_reset(state.imuID);
  }

  // determine the datum type
  sscanf(line, "%d", &datum_type);

  // process synced data (all three sensors)
  if (datum_type == 0) {
    IMU_data3    data3;
    sscanf(line, "%*d, %u, %hu, %hu, %hu, %hu, %hu, %hu, %hu, %hu, %hu", 
      &data3.t,
      &data3.g[0], &data3.g[1], &data3.g[2], 
      &data3.a[0], &data3.a[1], &data3.a[2],
      &data3.m[0], &data3.m[1], &data3.m[2]);
    IMU_engn_data3(0, &data3);
  }

  // process sensor datum (asynchrous feeds)
  else if (datum_type < 4) {
    IMU_datum    datum;
    sscanf(line, "%d, %u, %hu, %hu, %hu", 
      &datum.type, &datum.t, &datum.val[0], &datum.val[1], &datum.val[2]);
    IMU_engn_datum(0, &datum);
  }

  // exit function
  return status;
}


/******************************************************************************
* deconstructor for overall data parse block
******************************************************************************/

void dataIF_exit()
{
  state.exitThread = 1;
  while(!state.isExit)
    usleep(100); 
  if (!state.isCSV)
    close(dataIF_socket);
  else
    fclose(dataIF_file);
}


/******************************************************************************
* "continous run" function handle for recreating a thread 
******************************************************************************/

void* dataIF_run(
  void                   *id)
{
  // main processing loop
  state.exitThread = 0;
  state.isExit     = 0;
  while(!state.exitThread) {
    dataIF_process();
  }
  state.isExit     = 1;
  return 0;
}


/******************************************************************************
* utility function - displays error and exits program
******************************************************************************/

void dataIF_error(
  const char             *msg)
{
  perror(msg);
  exit(1);
}


/******************************************************************************
* extracts UDP line
******************************************************************************/

void dataIF_lineUPD(
  char                   *line,
  int                    line_size)
{
  int rc = recv(dataIF_socket, line, line_size, 0);
  line[rc] = (char)0;
}


/******************************************************************************
* extracts CSV line
******************************************************************************/

int dataIF_lineCSV(
  char                   *line,
  int                    line_size)
{
  // define local variables
  int                    status;
  struct timeval         time;
  double                 time_cur       = 0;
  double                 time_delt_sys  = 0;
  double                 time_delt_sen  = 0;
  char                   *results;

  // extracts one line (repeats when the file ends)
  results = fgets(line, line_size, dataIF_file);
  if (results == NULL) {
    if (config.isRepeat) {
      fseek(dataIF_file, 0, SEEK_SET);
      state.isFirstFrame = 1;
      results = fgets(line, line_size, dataIF_file);
    }
    status               = 1;
  } else {
    status               = 0;
  }

  // determine the datum type
  unsigned int sensor_time;
  sscanf(line, "%*d, %d", &sensor_time);

  // inject csv file delay 
  if (config.isRealtime && !state.isFirstFrame) {
    while (1) { 
      gettimeofday(&time, NULL);
      time_cur         = time.tv_sec * 1000000 + time.tv_usec;
      time_delt_sys    = time_cur - state.timeInitSys;
      time_delt_sen    = (sensor_time - state.timeInitSen) * 10;
      if (time_delt_sen - time_delt_sys > 0) 
        usleep(time_delt_sen - time_delt_sys); 
      else
        break;
    }
  }

  // initialize system and sensor time
  if (state.isFirstFrame) {
    gettimeofday(&time, NULL);
    state.timeInitSys  = time.tv_sec * 1000000 + time.tv_usec;
    state.timeInitSen  = sensor_time;
    state.isFirstFrame = 0;
  }

  // exit function
  return status;
}

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
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <errno.h>
#include "IMU_file.h"
#include "IMU_math.h"
#include "dataIF.h"

// define the data interface structures
dataIF_ctrl              ctrl;
dataIF_sensor            sensor;
dataIF_state             state;

// define internal/external variables
int                      data_socket;
FILE*                    csv_file;

// define internal/utility functions
void dataIF_error(const char *msg);
void dataIF_lineUPD(char* line);
void dataIF_lineCSV(char* line);


/******************************************************************************
* constructor for dataParse functions
******************************************************************************/

void dataIF_init(
  uint16_t               idRect, 
  uint16_t               idCore, 
  uint16_t               idPnts, 
  uint16_t               idAuto)
{
  // initialize and get handles to all IMU functions
  state.idRect           = idRect;
  state.idCore           = idCore;
  state.idPnts           = idPnts;
  state.idAuto           = idAuto;
  state.isFirstFrame     = 1;
  ctrl.isRealtime        = 0;
  ctrl.isExit            = 1;
}


/******************************************************************************
* get pointer to sensor structure
******************************************************************************/

void dataIF_getSensor(
  dataIF_sensor          **pntr)
{
  *pntr                  = &sensor;
}


/******************************************************************************
* creates UDP server
******************************************************************************/

void dataIF_startUDP(
  int                    portno)
{
  // define internal variables
  struct sockaddr_in     serv_addr;
  int                    status;
 
  // init IMU and data_stream state
  state.isCSV = 0;

  // open socket
  data_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (data_socket < 0) 
    dataIF_error("ERROR opening socket");

  // bind port 
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family       = AF_INET;
  serv_addr.sin_addr.s_addr  = INADDR_ANY;
  serv_addr.sin_port         = htons(portno);
  status = bind(data_socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
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
  csv_file    = fopen(filename, "r");
  if (!csv_file)
    dataIF_error("ERROR opening csv file"); 
}


/******************************************************************************
* main function for receiving/parsing data and updating IMU
******************************************************************************/

void dataIF_process()
{
  // define the variables
  const int              line_size = 256;
  char                   line[line_size];
  int                    datum_type; 

  // extract data line
  if (!state.isCSV)
    dataIF_lineUPD(line);
  else 
    dataIF_lineCSV(line);

  // determine the datum type
  sscanf(line, "%d, %f", &datum_type, &sensor.time);

  // process synced data (all three sensors)
  if (datum_type == 0) {
    IMU_data3    data3;
    data3.t      = sensor.time;
    data3.FOM    = &state.FOM;
    sscanf(line, "%*d, %*f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
      &sensor.gyroRaw[0], &sensor.gyroRaw[1], &sensor.gyroRaw[2],
      &sensor.acclRaw[0], &sensor.acclRaw[1], &sensor.acclRaw[2],
      &sensor.magnRaw[0], &sensor.magnRaw[1], &sensor.magnRaw[2]);
    IMU_rect_all(state.idRect, sensor.gyroRaw, sensor.acclRaw, sensor.magnRaw,
      sensor.gyroCor, sensor.acclCor, sensor.magnCor);
    memcpy(data3.g, sensor.gyroCor, sizeof(data3.g));
    memcpy(data3.a, sensor.acclCor, sizeof(data3.a));
    memcpy(data3.m, sensor.magnCor, sizeof(data3.m));
    IMU_pnts_data3(state.idPnts, &data3, &state.pnt);
    IMU_core_data3(state.idCore, &data3);
    IMU_auto_newFOM(state.idAuto, state.FOM, 3);
  }

  // process gyroscope data (async sensors)
  else if (datum_type == 1) {
    IMU_datum    datum;
    datum.type   = IMU_gyro;
    datum.t      = sensor.time;
    datum.FOM    = &state.FOM;
    sscanf(line, "%*d, %*f, %f, %f, %f", 
      &sensor.gyroRaw[0], &sensor.gyroRaw[1], &sensor.gyroRaw[2]);
    IMU_rect_gyro(state.idRect, sensor.gyroRaw, sensor.gyroCor);
    memcpy(datum.val, sensor.gyroCor, sizeof(datum.val));
    IMU_pnts_newGyro(state.idPnts, &datum, &state.pnt);
    IMU_core_newGyro(state.idCore, &datum);
    IMU_auto_newFOM(state.idAuto, state.FOM, 1);
  }

  // process accelerometer data (async sensors)
  else if (datum_type == 2) {
    IMU_datum    datum;
    datum.type   = IMU_accl;
    datum.t      = sensor.time;
    datum.FOM    = &state.FOM;
    sscanf(line, "%*d, %*f, %f, %f, %f", 
      &sensor.acclRaw[0], &sensor.acclRaw[1], &sensor.acclRaw[2]);
    IMU_rect_accl(state.idRect, sensor.acclRaw, sensor.acclCor);
    memcpy(datum.val, sensor.acclCor, sizeof(datum.val));
    IMU_pnts_newAccl(state.idPnts, &datum, &state.pnt);
    IMU_core_newAccl(state.idCore, &datum);
    IMU_auto_newFOM(state.idAuto, state.FOM, 1);
  }

  // process magnetometer data (async sensors)
  else if (datum_type == 3) {
    IMU_datum    datum;
    datum.type   = IMU_magn;
    datum.t      = sensor.time;
    datum.FOM    = &state.FOM;
    sscanf(line, "%*d, %*f, %f, %f, %f", 
      &sensor.magnRaw[0], &sensor.magnRaw[1], &sensor.magnRaw[2]);
    IMU_rect_magn(state.idRect, sensor.magnRaw, sensor.magnCor);
    memcpy(datum.val, sensor.acclCor, sizeof(datum.val));
    IMU_pnts_newMagn(state.idPnts, &datum, &state.pnt);
    IMU_core_newMagn(state.idCore, &datum);
    IMU_auto_newFOM(state.idAuto, state.FOM, 1);
  }
  
  // update IMU manual and automated calibration blocks
  if (state.pnt != NULL) 
    IMU_calb_pnts(state.idCalb, state.pnt);
}


/******************************************************************************
* deconstructor for overall data parse block
******************************************************************************/

void dataIF_exit()
{
  ctrl.exitThread = 1;
  while(!ctrl.isExit)
    usleep(100); 
  if (!state.isCSV)
    close(data_socket);
  else
    fclose(csv_file);
}


/******************************************************************************
* "continous run" function handle for recreating a thread 
******************************************************************************/

void* dataIF_run(void* id)
{
  // main processing loop
  ctrl.exitThread = 0;
  ctrl.isExit     = 0;
  while(!ctrl.exitThread) {
    dataIF_process();
  }
  ctrl.isExit     = 1;
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
  char                   *line)
{
  int rc = recv(data_socket, line, sizeof(line), 0);
  line[rc] = (char)0;
}


/******************************************************************************
* extracts CSV line
******************************************************************************/

void dataIF_lineCSV(
  char                   *line)
{
  // define local variables
  struct timeval         time;
  double                 time_cur       = 0;
  double                 time_delt_sys  = 0;
  double                 time_delt_sen  = 0;
  char                   *results;

  // extracts one line (repeats when the file ends)
  results = fgets(line, sizeof(line), csv_file);
  if (results == NULL) {
    fseek(csv_file, 0, SEEK_SET);
    state.isFirstFrame = 1;
    results = fgets(line, sizeof(line), csv_file);
  } 

  // determine the datum type
  float sensor_time;
  sscanf(line, "%*d, %f", &sensor_time);

  // inject delay if running csv file
  if (ctrl.isRealtime && !state.isFirstFrame) {
    while (1) { 
      gettimeofday(&time, NULL);
      time_cur         = time.tv_sec * 1000000 + time.tv_usec;
      time_delt_sys    = time_cur - state.timeInitSys;
      time_delt_sen    = sensor_time - state.timeInitSen;
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
}

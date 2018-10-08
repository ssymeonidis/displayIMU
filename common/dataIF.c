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
dataIF_data              data;
dataIF_state             state;

// define internal/external variables
int                      data_socket;
FILE*                    csv_file;

// define internal/utility functions
void dataIF_error(const char *msg);
void dataIF_lineUPD(char* line);
void dataIF_lineCSV(char* line);
void dataIF_copyData3(IMU_data3 *data3);
void dataIF_copyDatum(IMU_datum *datum, IMU_TYPE *val);


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

void dataIF_getPntr(
  dataIF_data            **pntr)
{
  *pntr                  = &data;
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
  sscanf(line, "%d, %f", &datum_type, &data.time);

  // process synced data (all three sensors)
  if (datum_type == 0) {
    IMU_data3    data3;
    data3.t      = data.time;
    data3.FOM    = state.FOM;
    sscanf(line, "%*d, %*f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
      &data.gRaw[0], &data.gRaw[1], &data.gRaw[2],
      &data.aRaw[0], &data.aRaw[1], &data.aRaw[2],
      &data.mRaw[0], &data.mRaw[1], &data.mRaw[2]);
    IMU_rect_data3(state.idRect, &data3, data.gRaw, data.aRaw, data.mRaw);
    IMU_pnts_data3(state.idPnts, &data3, &state.pnt);
    IMU_core_data3(state.idCore, &data3);
    IMU_auto_update(state.idAuto, data3.FOM, 3);
    dataIF_copyData3(&data3);
  }

  // process sensor datum (asynchrous feeds)
  else if (datum_type < 4) {
    IMU_datum    datum;
    IMU_TYPE     val[3];
    datum.type   = datum_type;
    datum.t      = data.time;
    datum.FOM    = state.FOM;
    sscanf(line, "%*d, %*f, %f, %f, %f", &val[0], &val[1], &val[2]);
    IMU_rect_datum(state.idRect, &datum, val);
    IMU_pnts_datum(state.idPnts, &datum, &state.pnt);
    IMU_core_datum(state.idCore, &datum);
    IMU_auto_update(state.idAuto, state.FOM, 1);
    dataIF_copyDatum(&datum, val);
  }
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


/******************************************************************************
* copies corrected and filtered data to internal structure
******************************************************************************/

void dataIF_copyData3(
  IMU_data3              *data3)
{
  memcpy(data.gCor, data3->g, sizeof(data.gCor));
  memcpy(data.aCor, data3->a, sizeof(data.aCor));
  memcpy(data.mCor, data3->m, sizeof(data.mCor));
}


/******************************************************************************
* copies corrected and filtered data to internal structure
******************************************************************************/

void dataIF_copyDatum(
  IMU_datum              *datum,
  IMU_TYPE               *val)
{
  if        (datum->type == IMU_gyro) {
    memcpy(data.gRaw, val,        sizeof(data.gRaw));
    memcpy(data.gCor, datum->val, sizeof(data.gCor));
  } else if (datum->type == IMU_accl) {
    memcpy(data.aRaw, val,        sizeof(data.aRaw));
    memcpy(data.aCor, datum->val, sizeof(data.aCor));
  } else if (datum->type == IMU_magn) {
    memcpy(data.mRaw, val,        sizeof(data.mRaw));
    memcpy(data.mCor, datum->val, sizeof(data.mCor));
  }
}

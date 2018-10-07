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
#include "dataParse.h"


// define the sensor data structure
dataParse_ctrl    ctrl;
dataParse_sensor  sensor;
dataParse_estim   estim;
dataParse_state   state;

// define internal/external variables
uint8_t                    is_log_data    = 0;
uint8_t                    is_csv_file    = 0;
uint8_t                    first_frame    = 1;
double                     time_init_sys  = 0;
double                     time_init_sen  = 0;
int                        data_socket;
FILE*                      csv_file;
FILE*                      log_file;


/******************************************************************************
* utility function - displays error and exits program
******************************************************************************/

void data_error(const char *msg)
{
  perror(msg);
  exit(1);
}


/******************************************************************************
* constructor for dataParse functions
******************************************************************************/

void data_init(
  char*  file_rect,
  char*  file_core,
  char*  file_pnts,
  char*  file_auto)
{
  // initialize and get handles to all IMU functions
  IMU_rect_init  (&state.idRect,  &state.configRect);
  IMU_core_init  (&state.idCore,  &state.configCore);
  IMU_pnts_init  (&state.idPnts,  &state.configPnts);
  IMU_auto_init  (&state.idAuto,  &state.configAuto);
  IMU_calb_init  (&state.idCalb);

  // read config structures from json files
  if (!file_rect) IMU_file_rectOpen  (file_rect, state.configRect);
  if (!file_core) IMU_file_coreOpen  (file_core, state.configCore);
  if (!file_pnts) IMU_file_pntsOpen  (file_pnts, state.configPnts);
  if (!file_auto) IMU_file_autoOpen  (file_auto, state.configAuto);

  // initialize the control structure
  ctrl.q_ref[0] = 1;
  ctrl.q_ref[1] = 0;
  ctrl.q_ref[2] = 0;
  ctrl.q_ref[3] = 0;
  ctrl.exit_thread = 0;
}


/******************************************************************************
* used to start logging results to csv file
******************************************************************************/

void data_start_log(const char* filename)
{
  log_file    = fopen(filename, "w+");
  if (!log_file)
    data_error("ERROR opening log file"); 
  setbuf(log_file, NULL);
  is_log_data = 1;
}


/******************************************************************************
* iniatilizes IMU and creates UDP server
******************************************************************************/

void data_start_UDP(int portno)
{
  // define internal variables
  struct sockaddr_in  serv_addr;
  int                 status;
 
  // init IMU and data_stream state
  is_csv_file = 0;

  // open socket
  data_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (data_socket < 0) 
    data_error("ERROR opening socket");

  // bind port 
  bzero((char *)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family       = AF_INET;
  serv_addr.sin_addr.s_addr  = INADDR_ANY;
  serv_addr.sin_port         = htons(portno);
  status = bind(data_socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr));
  if (status < 0) 
    data_error("ERROR on binding");
}


/******************************************************************************
* initializes IMU and creates CSV file reader
******************************************************************************/

void data_start_CSV(const char* filename)
{
  // init IMU and data_stream state
  is_csv_file = 1;

  // fopen file  
  csv_file    = fopen(filename, "r");
  if (!csv_file)
    data_error("ERROR opening csv file"); 
}


/******************************************************************************
* main function for receiving/parsing data and updating IMU
******************************************************************************/

void data_process_datum()
{
  // define the variables
  const int         line_size      = 256;
  char              line[line_size];
  int               datum_type; 
  struct timeval    time;
  double            time_cur       = 0;
  double            time_delt_sys  = 0;
  double            time_delt_sen  = 0;
  int               rc;
  char              *results;

  // extract data line
  if (!is_csv_file) {
    rc = recv(data_socket, line, sizeof(line), 0);
    line[rc] = (char)0;
  } else {
    results = fgets(line, sizeof(line), csv_file);
    if (results == NULL) {
      fseek(csv_file, 0, SEEK_SET);
      first_frame = 1;
      results = fgets(line, sizeof(line), csv_file);
    } 
  }

  // determine the datum type
  sscanf(line, "%d, %f", &datum_type, &sensor.time);

  // inject delay if running csv file
  if (is_csv_file && !first_frame) {
    while (1) { 
      gettimeofday(&time, NULL);
      time_cur         = time.tv_sec * 1000000 + time.tv_usec;
      time_delt_sys    = time_cur - time_init_sys;
      time_delt_sen    = sensor.time - time_init_sen;
      if (time_delt_sen - time_delt_sys > 0) 
        usleep(time_delt_sen - time_delt_sys); 
      else
        break;
    }
  }

  // initialize system and sensor time
  if (first_frame) {
    gettimeofday(&time, NULL);
    time_init_sys  = time.tv_sec * 1000000 + time.tv_usec;
    time_init_sen  = sensor.time;
    first_frame    = 0;
  }

  // process synced data (all three sensors)
  if (datum_type == 0) {
    sscanf(line, "%*d, %*f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
      &sensor.gyroRaw[0], &sensor.gyroRaw[1], &sensor.gyroRaw[2],
      &sensor.acclRaw[0], &sensor.acclRaw[1], &sensor.acclRaw[2],
      &sensor.magnRaw[0], &sensor.magnRaw[1], &sensor.magnRaw[2]);
    IMU_rect_all(state.idRect, sensor.gyroRaw, sensor.acclRaw, sensor.magnRaw,
      sensor.gyroCor, sensor.acclCor, sensor.magnCor);
    IMU_pnts_newAll(state.idPnts, sensor.time, sensor.gyroRaw, 
      sensor.acclRaw, sensor.magnRaw, &estim.pnt); estim.pnt = NULL; 
    if (estim.pnt != NULL) 
      IMU_calb_pnts(state.idCalb, estim.pnt, &estim.FOMcalib); 
    IMU_auto_newAll(state.idCore, sensor.time, sensor.gyroCor, 
      sensor.acclCor, sensor.magnCor); 
    IMU_core_newAll(state.idCore, sensor.time, sensor.gyroCor, sensor.acclCor, 
      sensor.magnCor, estim.FOMcore);
  }

  // process gyroscope data (async sensors)
  else if (datum_type == 1) {
    sscanf(line, "%*d, %*f, %f, %f, %f", 
      &sensor.gyroRaw[0], &sensor.gyroRaw[1], &sensor.gyroRaw[2]);
    IMU_rect_gyro(state.idRect, sensor.gyroRaw, sensor.gyroCor);
    IMU_pnts_newGyro(state.idPnts, sensor.time, sensor.gyroRaw, &estim.pnt);
    if (estim.pnt != NULL) 
      IMU_calb_pnts(state.idCalb, estim.pnt, &estim.FOMcalib); 
    IMU_auto_newGyro(state.idAuto, sensor.time, sensor.gyroCor);
    IMU_core_newGyro(state.idCore, sensor.time, sensor.gyroCor, estim.FOMcore);
  }

  // process accelerometer data (async sensors)
  else if (datum_type == 2) {
    sscanf(line, "%*d, %*f, %f, %f, %f", 
      &sensor.acclRaw[0], &sensor.acclRaw[1], &sensor.acclRaw[2]);
    IMU_rect_accl(state.idRect, sensor.acclRaw, sensor.acclCor);
    IMU_pnts_newAccl(state.idPnts, sensor.time, sensor.acclRaw, &estim.pnt);
    if (estim.pnt != NULL) 
      IMU_calb_pnts(state.idCalb, estim.pnt, &estim.FOMcalib); 
    IMU_auto_newAccl(state.idAuto, sensor.time, sensor.acclCor);
    IMU_core_newAccl(state.idCore, sensor.time, sensor.acclCor, estim.FOMcore);
  }

  // process magnetometer data (async sensors)
  else if (datum_type == 3) {
    sscanf(line, "%*d, %*f, %f, %f, %f", 
      &sensor.magnRaw[0], &sensor.magnRaw[1], &sensor.magnRaw[2]);
    IMU_rect_magn(state.idRect, sensor.magnRaw, sensor.magnCor);
    IMU_pnts_newMagn(state.idPnts, sensor.time, sensor.magnRaw, &estim.pnt);
    if (estim.pnt != NULL) 
      IMU_calb_pnts(state.idCalb, estim.pnt, &estim.FOMcalib); 
    IMU_auto_newMagn(state.idAuto, sensor.time, sensor.magnCor);
    IMU_core_newMagn(state.idCore, sensor.time, sensor.magnCor, estim.FOMcore);
  }
  
  // get estimates
  IMU_core_estmQuat(state.idCore, estim.q_org);
  IMU_core_estmAccl(state.idCore, estim.move);
  IMU_math_applyRef(estim.q_org, ctrl.q_ref, estim.q);
  IMU_math_calcEuler(estim.q_org, estim.ang);
  IMU_auto_newFOM(state.idAuto, estim.FOMcore, 1);   
  
  // write state to log file
  if (is_log_data)
    fprintf(log_file, "%f, %f, %f, %f, %f, %f, %f, %f, ",
      estim.q_org[0], estim.q_org[1], estim.q_org[2], estim.q_org[3],
      estim.q[0],     estim.q[1],     estim.q[2],     estim.q[3]);
    fprintf(log_file, "%f, %f, %f, %f, %f, %f\n",
      estim.ang[0],   estim.ang[1],   estim.ang[2],
      estim.move[0],  estim.move[1],  estim.move[2]);
}


/******************************************************************************
* deconstructor for overall data parse block
******************************************************************************/

void data_close()
{
  if (!is_csv_file)
    close(data_socket);
  else
    fclose(csv_file);
  if (is_log_data)
    fclose(log_file);
}


/******************************************************************************
* "continous run" function handle for recreating a thread 
******************************************************************************/

void* data_run(void* id)
{
  // main processing loop
  while(!ctrl.exit_thread) {
    data_process_datum();
  }
  return 0;
}

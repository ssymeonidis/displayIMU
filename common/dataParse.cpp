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
#include "dataParse.h"
#include "IMU_util_math.h"

// define the sensor data structure
dataParse_sensor    sensor;
dataParse_estim     estim;
IMU_core_FOM        FOM;
const int           buffer_size    = 100;
float               buffer[buffer_size][15];
int                 buffer_index   = 0;

// define internal/external variables
bool                is_log_data    = false;
bool                is_csv_file    = false;
bool                first_frame    = true;
double              time_init_sys  = 0;
double              time_init_sen  = 0;
int                 data_socket;
FILE*               csv_file;
FILE*               log_file;


/******************************************************************************
* utility function - displays error and exits program
******************************************************************************/

void data_error(const char *msg)
{
  perror(msg);
  exit(1);
}


/******************************************************************************
* used to initiate logging data to csv file
******************************************************************************/

void data_init_log(const char* filename)
{
  log_file    = fopen(filename, "w+");
  if (!log_file)
    data_error("ERROR opening log file"); 
  setbuf(log_file, NULL);
  is_log_data = true;
}


/******************************************************************************
* iniatilizes IMU and creates UDP server
******************************************************************************/

void data_init_UDP(int portno)
{
  // define internal variables
  sockaddr_in  serv_addr;
  int          status;
 
  // init IMU and data_stream state
  is_csv_file = false;

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
* iniatilizes IMU and creates CSV file reader
******************************************************************************/

void data_init_CSV(const char* filename)
{
  // init IMU and data_stream state
  is_csv_file = true;

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
  const int  line_size      = 256;
  char       line[line_size];
  int        datum_type; 
  float*     state;
  timeval    time;
  double     time_cur       = 0;
  double     time_delt_sys  = 0;
  double     time_delt_sen  = 0;
  int        rc;
  char       *results;

  // extract data line
  if (is_csv_file == false) {
    rc = recv(data_socket, line, sizeof(line), 0);
    line[rc] = (char)NULL;
  } else {
    results = fgets(line, sizeof(line), csv_file);
    if (results == NULL) {
      fseek(csv_file, 0, SEEK_SET);
      first_frame = true;
      results = fgets(line, sizeof(line), csv_file);
    } 
  }

  // determine the datum type
  sscanf(line, "%d, %f", &datum_type, &sensor.lastTime);

  // inject delay if running csv file
  if (is_csv_file == true && first_frame == false) {
    while (1) { 
      gettimeofday(&time, NULL);
      time_cur         = time.tv_sec * 1000000 + time.tv_usec;
      time_delt_sys    = time_cur - time_init_sys;
      time_delt_sen    = sensor.lastTime - time_init_sen;
      if (time_delt_sen - time_delt_sys > 0) 
        usleep(time_delt_sen - time_delt_sys); 
      else
        break;
    }
  }

  // initialize system and sensor time
  if (first_frame == true) {
    gettimeofday(&time, NULL);
    time_init_sys  = time.tv_sec * 1000000 + time.tv_usec;
    time_init_sen  = sensor.lastTime;
    first_frame    = false;
  }

  // process synced data (all three sensors)
  if (datum_type == 0) {
    sscanf(line, "%*d, %*f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
      &sensor.gyroRaw[0], &sensor.gyroRaw[1], &sensor.gyroRaw[2],
      &sensor.acclRaw[0], &sensor.acclRaw[1], &sensor.acclRaw[2],
      &sensor.magnRaw[0], &sensor.magnRaw[1], &sensor.magnRaw[2]);
    IMU_correct_all(0, sensor.gyroRaw, sensor.acclRaw, sensor.magnRaw,
      sensor.gyroCor, sensor.acclCor, sensor.magnCor);
    state = IMU_core_estmAll(0, sensor.lastTime, sensor.gyroCor, 
      sensor.acclCor, sensor.magnCor, &FOM);
    IMU_util_calcEuler(state, estim.ang);
  }

  // process gyroscope data (async sensors)
  else if (datum_type == 1) {
    sscanf(line, "%*d, %*f, %f, %f, %f", 
      &sensor.gyroRaw[0], &sensor.gyroRaw[1], &sensor.gyroRaw[2]);
    IMU_correct_gyro(0, sensor.gyroRaw, sensor.gyroCor);
    state = IMU_core_estmGyro(0, sensor.lastTime, sensor.gyroCor, &FOM);
    IMU_util_calcEuler(state, estim.ang);
  }

  // process accelerometer data (async sensors)
  else if (datum_type == 2) {
    sscanf(line, "%*d, %*f, %f, %f, %f", 
      &sensor.acclRaw[0], &sensor.acclRaw[1], &sensor.acclRaw[2]);
    IMU_correct_accl(0, sensor.acclRaw, sensor.acclCor);
    state = IMU_core_estmAccl(0, sensor.lastTime, sensor.acclCor, &FOM);
    IMU_util_calcEuler(state, estim.ang);
  }

  // process magnetometer data (async sensors)
  else if (datum_type == 3) {
    sscanf(line, "%*d, %*f, %f, %f, %f", 
      &sensor.magnRaw[0], &sensor.magnRaw[1], &sensor.magnRaw[2]);
    IMU_correct_magn(0, sensor.magnRaw, sensor.magnCor);
    state = IMU_core_estmMagn(0, sensor.lastTime, sensor.magnCor, &FOM);
    IMU_util_calcEuler(state, estim.ang);
  }
}


/******************************************************************************
* deconstructor for overall data parse block
******************************************************************************/

void data_close()
{
  if (is_csv_file == false)
    close(data_socket);
  else
    fclose(csv_file);
  if (is_log_data == true)
    fclose(log_file);
}


/******************************************************************************
* "continous run" function handle for recreating a thread 
******************************************************************************/

void *data_run(void*)
{
  // main processing loop
  while(1) {
    data_process_datum();
  }
}

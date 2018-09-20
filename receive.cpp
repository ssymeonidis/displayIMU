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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <errno.h>
#include "framesPerSecond.h"
#include "receive.h"

// define the sensor data structure
const int           sensor_buffer_size  = 100;
float               sensor_buffer[sensor_buffer_size][15];
int                 sensor_buffer_index = 0;
displayIMU_metrics  sensor_buffer_metrics;
int                 sensor_IMU_type     = 1; 
int                 sensor_IMU_reset    = 0;
int                 sensor_IMU_set_ref  = 0;
int                 sensor_IMU_calib    = 0;

// define internal/external variables
int                 sensor_data_socket;
FILE*               sensor_file;
const char*         sensor_filename     = "IMU.csv";


void sensor_data_error(const char *msg)
{
  perror(msg);
  exit(1);
}


void sensor_data_init(int portno)
{
  // init IMU
  displayIMU_init();
  sensor_file = fopen(sensor_filename, "w+");
  if (!sensor_file)
    fprintf(stderr, "Failed to open %s: %s\n", sensor_filename, strerror(errno)); 
  setbuf(sensor_file, NULL);

  // open socket
  sensor_data_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (sensor_data_socket < 0) 
    sensor_data_error("ERROR opening socket");

  // bind port 
  struct sockaddr_in serv_addr;
  bzero((char *) &serv_addr, sizeof(serv_addr));
  serv_addr.sin_family       = AF_INET;
  serv_addr.sin_addr.s_addr  = INADDR_ANY;
  serv_addr.sin_port         = htons(portno);
  if (bind(sensor_data_socket, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    sensor_data_error("ERROR on binding");

  // initialize the first elements to be empty
  sensor_buffer[0][0]  = 0.0;
  sensor_buffer[0][1]  = 0.0;
  sensor_buffer[0][2]  = 0.0;
  sensor_buffer[0][3]  = 0.0;
  sensor_buffer[0][4]  = 0.0;
  sensor_buffer[0][5]  = 1.0;
  sensor_buffer[0][6]  = 1.0;
  sensor_buffer[0][7]  = 0.0;
  sensor_buffer[0][8]  = 0.0;
  sensor_buffer[0][9]  = 0.0;
  sensor_buffer[0][10] = 0.0;
  sensor_buffer[0][11] = 0.0;
  sensor_buffer[0][12] = 0.0;
  sensor_buffer[0][13] = 0.0;
  sensor_buffer[0][14] = 0.0;
}


void *sensor_data_run(void*)
{
  // define the variables
  char  socket_buffer[256];
  float sensor_buffer_corrected[9];
  int   rc;
  int   index;

  // create the fps counter
  framesPerSecond fps;

  // main processing loop
  while(1) {
    // receive UDP data
    rc = recv(sensor_data_socket, socket_buffer, sizeof(socket_buffer), 0);
    socket_buffer[rc] = (char)NULL;
    
    // store the results into ring buffer
    index      = sensor_buffer_index+1;
    if (index >= sensor_buffer_size)
      index    = 0; 
    sscanf(socket_buffer, "%f, %f, %f, %f, %f, %f, %f, %f, %f", 
      &(sensor_buffer[index][0]), &(sensor_buffer[index][1]), &(sensor_buffer[index][2]),
      &(sensor_buffer[index][3]), &(sensor_buffer[index][4]), &(sensor_buffer[index][5]),
      &(sensor_buffer[index][6]), &(sensor_buffer[index][7]), &(sensor_buffer[index][8]));
    sensor_buffer_index = index;

    // perform the specified IMU
    if (sensor_IMU_type == 1) {
      displayIMU_corAll(&(sensor_buffer[index][3]),
                        &(sensor_buffer[index][0]),
                        &(sensor_buffer[index][6]),
                        &(sensor_buffer_corrected[3]),
                        &(sensor_buffer_corrected[0]),
                        &(sensor_buffer_corrected[6])); 
      displayIMU_estmAll(NULL,
                 &(sensor_buffer_corrected[3]),
                 &(sensor_buffer_corrected[0]),
                 &(sensor_buffer_corrected[6]),
                 &(sensor_buffer[index][9]),
                 &(sensor_buffer[index][12]),
                 &sensor_buffer_metrics);
      fprintf(sensor_file, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,",
              sensor_buffer[index][6],
              sensor_buffer[index][7],
              sensor_buffer[index][8],
              sensor_buffer[index][0],
              sensor_buffer[index][1],
              sensor_buffer[index][2],
              sensor_buffer[index][3],
              sensor_buffer[index][4],
              sensor_buffer[index][5]);
      fprintf(sensor_file, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
              sensor_buffer[index][9],
              sensor_buffer[index][10],
              sensor_buffer[index][11],
              sensor_buffer[index][12],
              sensor_buffer[index][13],
              sensor_buffer[index][14]);
    }
    sensor_IMU_set_ref = 0;
    sensor_IMU_reset   = 0;
    sensor_IMU_calib   = 0; 

    // update the frames per second
    fps.get();
  }

  // exit function
  close(sensor_data_socket);
  fclose(sensor_file);
  return NULL;
}

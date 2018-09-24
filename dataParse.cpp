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
#include <errno.h>
#include "dataParse.h"

// define the sensor data structure
const int           buffer_size  = 100;
float               buffer[buffer_size][15];
int                 buffer_index = 0;
displayIMU_metrics  FOM;

// define internal/external variables
bool                is_log_data   = false;
bool                is_csv_file   = false;
int                 data_socket;
FILE*               csv_file;
FILE*               log_file;


void data_error(const char *msg)
{
  perror(msg);
  exit(1);
}


void log_data(const char* filename)
{
  log_file    = fopen(filename, "w+");
  if (!log_file)
    data_error("ERROR opening log file"); 
  setbuf(log_file, NULL);
  is_log_data = true;
}


void data_init_UDP(int portno)
{
  // define internal variables
  sockaddr_in  serv_addr;
  int          status;
 
  // init IMU and data_stream state
  displayIMU_init();
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


void data_init_CSV(const char* filename)
{
  // init IMU and data_stream state
  displayIMU_init();
  is_csv_file = true;

  // fopen file  
  csv_file    = fopen(filename, "r");
  if (!csv_file)
    data_error("ERROR opening csv file"); 
}


void *data_run(void*)
{
  // define the variables
  const int  line_size = 256;
  char       line[line_size];
  float buffer_corrected[9];
  int   rc;
  int   index;
int                 sensor_type   = 1; 

  // main processing loop
  while(1) {

    // extract data line
    if (is_csv_file == false) {
      rc = recv(data_socket, line, sizeof(line), 0);
      line[rc] = (char)NULL;
    } else {
      fgets(line, sizeof(line), csv_file);
      if (line == NULL)
        break;
    } 
    
    // store the results into ring buffer
    index      = buffer_index+1;
    if (index >= buffer_size)
      index    = 0; 
    sscanf(line, "%*f, %*f, %f, %f, %f, %f, %f, %f, %f, %f, %f", 
      &buffer[index][0], &buffer[index][1], &buffer[index][2],
      &buffer[index][3], &buffer[index][4], &buffer[index][5],
      &buffer[index][6], &buffer[index][7], &buffer[index][8]);
    buffer_index = index;

    // perform the specified IMU
    if (sensor_type == 1) {
      displayIMU_corAll(&buffer[index][0],
                        &buffer[index][3],
                        &buffer[index][6],
                        &buffer_corrected[0], &buffer_corrected[3], &buffer_corrected[6]); 
      displayIMU_estmAll(NULL,
                 &buffer_corrected[0],
                 &buffer_corrected[3],
                 &buffer_corrected[6],
                 &buffer[index][9],
                 &buffer[index][12],
                 &FOM);
      if (is_log_data == true) {
      fprintf(log_file, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,",
              buffer[index][6],
              buffer[index][7],
              buffer[index][8],
              buffer[index][0],
              buffer[index][1],
              buffer[index][2],
              buffer[index][3],
              buffer[index][4],
              buffer[index][5]);
      fprintf(log_file, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
              buffer[index][9],
              buffer[index][10],
              buffer[index][11],
              buffer[index][12],
              buffer[index][13],
              buffer[index][14]);
      }
    }
  }

  // exit function
  close(data_socket);
  fclose(log_file);
  return NULL;
}

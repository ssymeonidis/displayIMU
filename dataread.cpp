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

#include "dataread.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <csv.h>

// define the sensor data structure
const int           csv_buffer_size     = 500000;
float               csv_buffer[csv_buffer_size][16];
int                 csv_buffer_index    = 0;
displayIMU_metrics  csv_buffer_metrics;

// define control constants
int                 csv_IMU_reset       = 1;
int                 csv_IMU_set_ref     = 0;
int                 csv_IMU_calib       = 0;
int                 csv_IMU_first_run   = 1;

// define internal/external variables
FILE*               output_file;
const char*         output_file_ext     = ".IMU.csv";
int                 fieldCur            = 0;
int                 indexMax            = 0;


void cvs_field(void *s, size_t, void*) {
  double val    = atof((char*)s);
  csv_buffer[indexMax][fieldCur] = val;
  fieldCur++;
}


void cvs_EOL(int, void*) {
  fieldCur    = 0;
  indexMax   += 1;
}


void csv_data_init(char* filename)
{
  // configure IMU and data output
  displayIMU_init();
  char output_file_name[200];
  strcpy(output_file_name, filename);
  strcat(output_file_name, output_file_ext);
  output_file = fopen(output_file_name, "w+");
  if (!output_file)
    fprintf(stderr, "Failed to open %s: %s\n", output_file_name, strerror(errno)); 
  // setbuf(output_file, NULL); 

  // declare the csv read variables
  FILE*              fp;
  struct csv_parser  p;
  char               buf[1024];
  size_t             bytes_read;
  unsigned char      options = CSV_APPEND_NULL;

  // open the csv file
  fp = fopen(filename, "rb");
  if (!fp) 
    fprintf(stderr, "Failed to open %s: %s\n", filename, strerror(errno));
  else
    printf("opening file %s\n", filename);

  // create the csv parsing structure
  csv_init(&p, options);

  // parse the data using fixed-sized memory segments
  while ((bytes_read=fread(buf, 1, 1024, fp)) > 0)
    if (csv_parse(&p, buf, bytes_read, cvs_field, cvs_EOL, NULL) != bytes_read) 
      fprintf(stderr, "Error while parsing file: %s\n", csv_strerror(csv_error(&p)));
  csv_fini(&p, cvs_field, cvs_EOL, NULL);
  if (ferror(fp)) 
    fprintf(stderr, "Error while reading file %s\n", filename);

  // close all open handles
  printf("finished reading file\n");
  csv_free(&p);
  fclose(fp);
}


void *csv_data_run(void*)
{
  float csv_buffer_corrected[15];
  while(1) {
    //usleep(500000);
    usleep(30000);
    usleep(300);
    if (csv_buffer_index >= indexMax) {
      if (csv_IMU_first_run != 0 && output_file != NULL)
        fclose(output_file);
      csv_buffer_index   = 0;
      csv_IMU_reset      = 1;
      csv_IMU_first_run  = 0;
    } else  {
      csv_buffer_index++;
    }
    displayIMU_corAll(&(csv_buffer[csv_buffer_index][7]),
                      &(csv_buffer[csv_buffer_index][4]),
                      &(csv_buffer[csv_buffer_index][1]),
                      &(csv_buffer_corrected[7]),
                      &(csv_buffer_corrected[4]),
                      &(csv_buffer_corrected[1]));
    displayIMU_estmAll(NULL,
               &(csv_buffer_corrected[7]),
               &(csv_buffer_corrected[4]),
               &(csv_buffer_corrected[1]),
               &(csv_buffer[csv_buffer_index][10]),
               &(csv_buffer[csv_buffer_index][13]),
               &csv_buffer_metrics);
    if (csv_IMU_first_run != 0) {
      fprintf(output_file, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,",
              csv_buffer[csv_buffer_index][0],
              csv_buffer[csv_buffer_index][1],
              csv_buffer[csv_buffer_index][2],
              csv_buffer[csv_buffer_index][3],
              csv_buffer[csv_buffer_index][4],
              csv_buffer[csv_buffer_index][5],
              csv_buffer[csv_buffer_index][6],
              csv_buffer[csv_buffer_index][7],
              csv_buffer[csv_buffer_index][8],
              csv_buffer[csv_buffer_index][9]);
      fprintf(output_file, "%.8f,%.8f,%.8f,%.8f,%.8f,%.8f\n",
              csv_buffer[csv_buffer_index][10],
              csv_buffer[csv_buffer_index][11],
              csv_buffer[csv_buffer_index][12],
              csv_buffer[csv_buffer_index][13],
              csv_buffer[csv_buffer_index][14],
              csv_buffer[csv_buffer_index][15]);
    }
    csv_IMU_set_ref    = 0;
    csv_IMU_reset      = 0;
    csv_IMU_calib      = 0;
  }
  return NULL;
}

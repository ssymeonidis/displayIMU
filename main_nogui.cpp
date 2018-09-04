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

#include "MARG.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <csv.h>

// define the sensor data structure
const int    csv_buffer_size     = 500000;
float        csv_buffer[csv_buffer_size][16];
int          csv_buffer_index    = 0;

// define control constants
int          csv_IMU_reset       = 1;
int          csv_IMU_set_ref     = 0;
int          csv_IMU_calib       = 0;

// define internal/external variables
FILE*        output_file;
const char*  output_file_ext     = ".IMU.csv";
int          fieldCur            = 0;
int          indexMax            = 0;


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
  initMARG(1);
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


void param_data_init(char* filename)
{
  // declare the variables
  FILE*   fid;
  int     pass;

  // read the variables
  fid = fopen(filename, "r");
  if (fid == NULL)
    printf("cannot open file %s\n", filename);
  else {
    pass = fscanf(fid, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
      &aBiasX, &aBiasY, &aBiasZ, &G, &mBiasX, &mBiasY, &mBiasZ,
      &M, &ang, &gBiasX, &gBiasY, &gBiasZ, &gyro_scale);
    if (pass != 13)
      fprintf(stderr, "Error while reading file %s\n", filename);
    fclose(fid);
  }
}


int main(int argc, char* argv[])
{
  // verify command line inputs
  if (argc < 3)
    fprintf(stderr, "Error: expecting %s data_file.csv config_file.csv", argv[0]);

  // read inputs files
  csv_data_init(argv[1]);
  param_data_init(argv[2]);

  // main loop 
  while(csv_buffer_index < indexMax) {
    updateMARG(&(csv_buffer[csv_buffer_index][7]),
               &(csv_buffer[csv_buffer_index][4]),
               &(csv_buffer[csv_buffer_index][1]),
               &(csv_buffer[csv_buffer_index][10]),
               &(csv_buffer[csv_buffer_index][13]),
               csv_IMU_set_ref,
               csv_IMU_reset,
               csv_IMU_calib);
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
    csv_buffer_index++;
    csv_IMU_set_ref    = 0;
    csv_IMU_reset      = 0;
    csv_IMU_calib      = 0;
  }
  return 0;
}

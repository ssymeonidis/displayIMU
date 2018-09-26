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
#include <string.h>
#include "fileUtils.h"

// calib structure parsing inputs
static const int   calib_size = 9;
static const char* calib_name[] = {
  "gBias",
  "gMult",
  "aBias",
  "aMult",
  "mBias",
  "mMult",
  "aMag",
  "mMag",
  "mAng"
};
enum calib_enum {
  gBias   = 0,
  gMult   = 1,
  aBias   = 2,
  aMult   = 3,
  mBias   = 4,
  mMult   = 5,
  aMag    = 6,
  mMag    = 7,
  mAng    = 8
};

// config struct parsing inputs
static const int   config_size = 13;
static const char* config_name[] = {
  "isGyro", 
  "isAccl",
  "isMagn",
  "isFltr",
  "isTear",
  "isAcclEstm",
  "isAutocal",
  "aWeight",
  "mWeight",
  "acclAlpha",
  "autocalAlpha1",
  "autocalAlpha2",
  "gAutocalThresh"
};
enum config_enum {
  isGyro          = 0,
  isAccl          = 1,
  isMagn          = 2,
  isFltr          = 3,
  isTear          = 4,
  isAcclEstm      = 5,
  isAutocal       = 6,
  aWeight         = 7,
  mWeight         = 8,
  acclAlpha       = 9,
  autocalAlpha1   = 10,
  autocalAlpha2   = 11,
  gAutocalThresh  = 12
};

// buffers used for parsing
static const int line_size = 128;
static char line[line_size];
static char temp[line_size];


/******************************************************************************
* utility function - gets a line and seperates field from arguments
******************************************************************************/

int get_line(FILE *file, char** field, char** args)
{
  fgets(line, line_size, file);
  *field = strtok(line, ":"); 
  sscanf(*field, "%s", temp);
  if (strcmp(temp, "{") == 0)
    return 1;
  if (strcmp(temp, "}") == 0)
    return 2;
  if (temp == NULL)
    return -2;
  *args = strtok(NULL, "\n");
  strtok(*field, "\"");
  *field = strtok(NULL, "\""); 
  return 0;
} 


/******************************************************************************
* utility function - matches field string with respective enum type
******************************************************************************/

int get_field(char* field, const char* names[], int size)
{
  int   i;
  for (i=0; i<size; i++) {
    if (strcmp(field, names[i]) == 0)
      return i;
  } 
  return -1; 
}


/******************************************************************************
* utility function - gets an array of comma seperated floats 
******************************************************************************/

int get_floats(char* args, float* vals, int size)
{
  char* cur;
  int   i;
  strtok(args, "[");
  for (i=0; i<size; i++) {
    cur = strtok(NULL, ",]"); 
    if (cur == NULL)
      return i;
    sscanf(cur, "%f", &vals[i]);
  }  
  return size; 
}


/******************************************************************************
* utility function - converts true/false string into boolean 
******************************************************************************/

int get_bool(char* args, bool* val)
{
  char temp[16];
  sscanf(args, "%s", temp);
  if      (strcmp(temp, "true")   == 0 ||
           strcmp(temp, "true,")  == 0)
    *val = true;
  else if (strcmp(temp, "false")  == 0 ||
           strcmp(temp, "false,") == 0)
    *val = false;
  else
    return -3;
  return 0;
}


/******************************************************************************
* utility function - writes json float array (includes brackets and commas)
******************************************************************************/

void write_floats(FILE* file, float* vals, int size)
{
  fprintf(file, "[");
  for (int i=0; i<size-1; i++)
    fprintf(file, "%f, ", vals[i]);
  fprintf(file, "%f],\n", vals[size-1]); 
}


/******************************************************************************
* utility function - write boolean string
******************************************************************************/

void write_bool(FILE* file, bool val)
{
  if (val == true)
    fprintf(file, "true,\n");
  else
    fprintf(file, "false,\n");
}


/******************************************************************************
* reads calibration json file into memory (structure)
******************************************************************************/

int displayIMU_readCalib(char* filename, displayIMU_calib *calib) 
{
  // define internal variables
  FILE*     file;
  char*     field;
  char*     args;
  int       type;
  int       status;

  // open json file containg config struct
  file = fopen(filename, "r");
  if (file == NULL)
    return -1;

  // main loop that parse json file line by line
  while (1) {
    // read line and parse field/args
    status = get_line(file, &field, &args);
    if (status == 1)
      continue;
    if (status > 1 || status < 0)
      break;

    // extract arguments for the specified field 
    type = get_field(field, calib_name, calib_size);
    if      (type == gBias) 
      get_floats(args, calib->gBias, 3);
    else if (type == gMult)
      get_floats(args, calib->gMult, 9);
    else if (type == aBias)
      get_floats(args, calib->aBias, 3);
    else if (type == aMult)
      get_floats(args, calib->aMult, 9);
    else if (type == mBias)
      get_floats(args, calib->mBias, 3);
    else if (type == mMult)
      get_floats(args, calib->mMult, 9);
    else if (type == aMag)
      sscanf(args, "%f", &calib->aMag);
    else if (type == mMag)
      sscanf(args, "%f", &calib->mMag);
    else if (type == mAng)
      sscanf(args, "%f", &calib->mAng);
  }

  // exit function
  fclose(file);
  return 0;   
}


/******************************************************************************
* writes calibration structure to a json file
******************************************************************************/

int displayIMU_writeCalib(char* filename, displayIMU_calib *calib)
{
  // define internal variables
  FILE*    file;

  // open file to contain json struct
  file = fopen(filename, "w");
  if (file == NULL)
    return -1;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"gBias\": ");  write_floats(file, calib->gBias, 3);
  fprintf(file, "  \"gMult\": ");  write_floats(file, calib->gMult, 9);
  fprintf(file, "  \"aBias\": ");  write_floats(file, calib->aBias, 3);
  fprintf(file, "  \"aMult\": ");  write_floats(file, calib->aMult, 9);
  fprintf(file, "  \"mBias\": ");  write_floats(file, calib->mBias, 3);
  fprintf(file, "  \"mMult\": ");  write_floats(file, calib->mMult, 9);
  fprintf(file, "  \"aMag\": %f,\n", calib->aMag);
  fprintf(file, "  \"mMag\": %f,\n", calib->mMag);
  fprintf(file, "  \"mAng\": %f\n",  calib->mAng);
  fprintf(file, "}");

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int displayIMU_readConfig(char* filename, displayIMU_config *config)
{
  // define internal variables
  FILE*     file;
  char*     field;
  char*     args;
  int       type;
  int       status;

  // open json file containg config struct
  file = fopen(filename, "r");
  if (file == NULL)
    return -1;

  // main loop that parse json file line by line
  while (1) {

    // read line and parse field/args
    status = get_line(file, &field, &args);
    if (status > 1 || status < 0)
      break;

    // extract arguments for the specified field
    type = get_field(field, config_name, config_size);
    if      (type == isGyro)
      get_bool(args, &config->isGyro);
    else if (type == isAccl)
      get_bool(args, &config->isAccl);
    else if (type == isMagn)
      get_bool(args, &config->isMagn);
    else if (type == isFltr)
      get_bool(args, &config->isFltr);
    else if (type == isTear)
      get_bool(args, &config->isTear);
    else if (type == isAcclEstm)
      get_bool(args, &config->isAcclEstm);
    else if (type == isAutocal)
      get_bool(args, &config->isAutocal);
    else if (type == aWeight)
      sscanf(args, "%f", &config->aWeight);
    else if (type == mWeight)
      sscanf(args, "%f", &config->mWeight);
    else if (type == acclAlpha)
      sscanf(args, "%f", &config->acclAlpha);
    else if (type == autocalAlpha1)
      sscanf(args, "%f", &config->autocalAlpha1);
    else if (type == autocalAlpha2)
      sscanf(args, "%f", &config->autocalAlpha2);
    else if (type == gAutocalThresh)
      sscanf(args, "%f", &config->gAutocalThresh);
  }

  // exit function
  fclose(file);
  return 0;  
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int displayIMU_writeConfig(char* filename, displayIMU_config *config)
{
  // define internal variables
  FILE*    file;

  // open file to contain json struct
  file = fopen(filename, "w");
  if (file == NULL)
    return -1;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"isGyro\": ");      write_bool(file, config->isGyro);
  fprintf(file, "  \"isAccl\": ");      write_bool(file, config->isAccl);
  fprintf(file, "  \"isMagn\": ");      write_bool(file, config->isMagn);
  fprintf(file, "  \"isFltr\": ");      write_bool(file, config->isFltr);
  fprintf(file, "  \"isTear\": ");      write_bool(file, config->isTear);
  fprintf(file, "  \"isAcclEstm\": ");  write_bool(file, config->isAcclEstm);
  fprintf(file, "  \"isAutocal\": ");   write_bool(file, config->isAutocal);
  fprintf(file, "  \"aWeight\": %f,\n",         config->aWeight);
  fprintf(file, "  \"mWeight\": %f,\n",         config->mWeight);
  fprintf(file, "  \"acclAlpha\": %f,\n",       config->acclAlpha);
  fprintf(file, "  \"autocalAlpha1\": %f,\n",   config->autocalAlpha1);
  fprintf(file, "  \"autocalAlpha2\": %f,\n",   config->autocalAlpha2);
  fprintf(file, "  \"gAutocalThreah\": %f\n",   config->gAutocalThresh);
  fprintf(file, "}");

  // exit function
  fclose(file);
  return 0;
}

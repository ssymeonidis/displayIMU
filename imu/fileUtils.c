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
static const int   config_size = 16;
static const char* config_name[] = {
  "isGyro", 
  "isAccl",
  "isMagn",
  "isFltr",
  "isTear",
  "isMove",
  "isFOM",
  "isAutocal",
  "gThreshVal",
  "gThreshTime",
  "aWeight",
  "aAlpha",
  "mWeight",
  "mAlpha",
  "moveAlpha",
  "autocalAlpha"
};
enum config_enum {
  isGyro          = 0,
  isAccl          = 1,
  isMagn          = 2,
  isFltr          = 3,
  isTear          = 4,
  isMove          = 5,
  isFOM           = 6,
  isAutocal       = 7,
  gThreshVal      = 8,
  gThreshTime     = 9,
  aWeight         = 10,
  aAlpha          = 11,
  mWeight         = 12,
  mAlpha          = 13,
  moveAlpha       = 14,
  autocalAlpha    = 15
};

// buffers used for parsing
static const int line_size = 128;
static char line[line_size];
static char temp[line_size];


/******************************************************************************
* utility function - gets a line and seperates field from arguments
******************************************************************************/

int displayIMU_getLine(FILE *file, char** field, char** args)
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
    fprintf(file, "%0.2f, ", vals[i]);
  fprintf(file, "%0.2f],\n", vals[size-1]); 
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
    status = displayIMU_getLine(file, &field, &args);
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
  fprintf(file, "  \"aMag\": %0.2f,\n", calib->aMag);
  fprintf(file, "  \"mMag\": %0.2f,\n", calib->mMag);
  fprintf(file, "  \"mAng\": %0.2f\n",  calib->mAng);
  fprintf(file, "}\n");

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
    status = displayIMU_getLine(file, &field, &args);
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
    else if (type == isMove)
      get_bool(args, &config->isMove);
    else if (type == isFOM)
      get_bool(args, &config->isFOM);
    else if (type == isAutocal)
      get_bool(args, &config->isAutocal);
    else if (type == gThreshVal)
      sscanf(args, "%f", &config->gThreshVal);
    else if (type == gThreshTime)
      sscanf(args, "%f", &config->gThreshTime);
    else if (type == aWeight)
      sscanf(args, "%f", &config->aWeight);
    else if (type == aAlpha)
      sscanf(args, "%f", &config->aAlpha);
    else if (type == mWeight)
      sscanf(args, "%f", &config->mWeight);
    else if (type == mAlpha)
      sscanf(args, "%f", &config->mAlpha);
    else if (type == moveAlpha)
      sscanf(args, "%f", &config->moveAlpha);
    else if (type == autocalAlpha)
      sscanf(args, "%f", &config->autocalAlpha);
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
  fprintf(file, "  \"isMove\": ");      write_bool(file, config->isMove);
  fprintf(file, "  \"isFOM\": ");       write_bool(file, config->isFOM);
  fprintf(file, "  \"isAutocal\": ");   write_bool(file, config->isAutocal);
  fprintf(file, "  \"gThreshVal\": %0.2f,\n",      config->gThreshVal);
  fprintf(file, "  \"gThreshTime\": %0.2f,\n",     config->gThreshTime);
  fprintf(file, "  \"aWeight\": %0.2f,\n",         config->aWeight);
  fprintf(file, "  \"aAlpha\": %0.2f,\n",          config->aAlpha);
  fprintf(file, "  \"mWeight\": %0.2f,\n",         config->mWeight);
  fprintf(file, "  \"mAlpha\": %0.2f,\n",          config->mAlpha);
  fprintf(file, "  \"moveAlpha\": %0.2f,\n",       config->moveAlpha);
  fprintf(file, "  \"autocalAlpha\": %0.2f,\n",    config->autocalAlpha);
  fprintf(file, "}\n");

  // exit function
  fclose(file);
  return 0;
}

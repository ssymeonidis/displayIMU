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
#include <string.h>
#include <stdint.h>
#include "IMU_file.h"

// calib structure parsing inputs
static const int   rect_config_size = 6;
static const char* rect_config_name[] = {
  "gBias",
  "gMult",
  "aBias",
  "aMult",
  "mBias",
  "mMult"
};
typedef enum {
  gBias   = 0,
  gMult   = 1,
  aBias   = 2,
  aMult   = 3,
  mBias   = 4,
  mMult   = 5
} rect_config_enum;

// config struct parsing inputs
static const int   core_config_size = 17;
static const char* core_config_name[] = {
  "isGyro", 
  "isAccl",
  "isMagn",
  "isStable",
  "isFOM",
  "isMove",
  "gThreshVal",
  "gThreshTime",
  "aWeight",
  "aMag",
  "aMagThresh",
  "mWeight",
  "mMag",
  "mMagThresh",
  "mAng",
  "mAngThresh",
  "moveAlpha"
};
typedef enum {
  isGyro            = 0,
  isAccl            = 1,
  isMagn            = 2,
  isStable          = 3,
  isFOM             = 4,
  isMove            = 5,
  gThreshVal        = 6,
  gThreshTime       = 7,
  aWeight           = 8,
  aMag              = 9,
  aMagThresh        = 10,
  mWeight           = 11,
  mMag              = 12,
  mMagThresh        = 13,
  mAng              = 14,
  mAngThresh        = 15,
  moveAlpha         = 16
} core_config_enum;

// calib structure parsing inputs
static const int   pnts_config_size = 9;
static const char* pnts_config_name[] = {
  "isAccl",
  "isMagn",
  "gAlpha",
  "gThreshVal",
  "gThreshTime",
  "aAlpha",
  "aThresh",
  "mAlpha",
  "mThresh"
};
typedef enum {
  pnts_isAccl      = 0,
  pnts_isMagn      = 1,
  pnts_gAlpha      = 2,
  pnts_gThreshVal  = 3,
  pnts_gThreshTime = 4,
  pnts_aAlpha      = 5,
  pnts_aThresh     = 6,
  pnts_mAlpha      = 7,
  pnts_mThresh     = 8
} pnts_config_enum;

// calib structure parsing inputs
static const int   auto_config_size = 7;
static const char* auto_config_name[] = {
  "isGyro",
  "isAccl",
  "isMagn",
  "gAlpha",
  "aAlpha",
  "mAlpha"
};
typedef enum {
  auto_isGyro      = 0,
  auto_isAccl      = 1,
  auto_isMagn      = 2,
  auto_gAlpha      = 3,
  auto_gThreshVal  = 4,
  auto_aAlpha      = 5,
  auto_mAlpha      = 6,
} auto_config_enum;

// buffers used for parsing
#define line_size 128
static char line[line_size];
static char temp[line_size];


/******************************************************************************
* utility function - gets an array of comma seperated floats 
******************************************************************************/

int get_floats(
  char                 *args, 
  float                *vals, 
  int                  size)
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

int get_bool(
  char                 *args, 
  uint8_t              *val)
{
  char temp[16];
  sscanf(args, "%s", temp);
  if      (strcmp(temp, "true")   == 0 ||
           strcmp(temp, "true,")  == 0)
    *val = 1;
  else if (strcmp(temp, "false")  == 0 ||
           strcmp(temp, "false,") == 0)
    *val = 0;
  else
    return IMU_FILE_INVALID_BOOL;
  return 0;
}


/******************************************************************************
* utility function - writes json float array (includes brackets and commas)
******************************************************************************/

void write_floats(
  FILE                 *file, 
  float                *vals, 
  int                  size)
{
  fprintf(file, "[");
  for (int i=0; i<size-1; i++)
    fprintf(file, "%0.2f, ", vals[i]);
  fprintf(file, "%0.2f],\n", vals[size-1]); 
}


/******************************************************************************
* utility function - write boolean string
******************************************************************************/

void write_bool(
  FILE                 *file, 
  uint8_t              val)
{
  if (val == 0)
    fprintf(file, "false,\n");
  else
    fprintf(file, "true,\n");
}


/******************************************************************************
* utility function - gets a line and seperates field from arguments
******************************************************************************/

int IMU_file_getLine(
  FILE                 *file, 
  char                 **field, 
  char                 **args)
{
  char *status; 
  status  = fgets(line, line_size, file);
  if (status == NULL)
    return IMU_FILE_UNEXPECTED_EOF;
  *field = strtok(line, ":"); 
  sscanf(*field, "%s", temp);
  if (strcmp(temp, "{") == 0)
    return 1;
  if (strcmp(temp, "}") == 0)
    return 2;
  if (temp == NULL)
    return IMU_FILE_MISSING_ARGS;
  *args = strtok(NULL, "\n");
  strtok(*field, "\"");
  *field = strtok(NULL, "\""); 
  return 0;
} 


/******************************************************************************
* utility function - matches field string with respective enum type
******************************************************************************/

int IMU_file_getField(
  char                  *field, 
  const char            *names[], 
  int                   size)
{
  int   i;
  for (i=0; i<size; i++) {
    if (strcmp(field, names[i]) == 0)
      return i;
  } 
  return IMU_FILE_INVALID_FIELD; 
}


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int IMU_file_readCore(
  char*                 filename,
  IMU_core_config       *config)
{
  // define internal variables
  FILE                  *file;
  char                  *field;
  char                  *args;
  core_config_enum      type;
  int                   status;

  // open json file containg config struct
  file = fopen(filename, "r");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // main loop that parse json file line by line
  while (1) {

    // read line and parse field/args
    status = IMU_file_getLine(file, &field, &args);
    if (status > 1 || status < 0)
      break;

    // extract arguments for the specified field
    type = IMU_file_getField(field, core_config_name, core_config_size);
    if      (type == isGyro)
      get_bool(args, &config->isGyro);
    else if (type == isAccl)
      get_bool(args, &config->isAccl);
    else if (type == isMagn)
      get_bool(args, &config->isMagn);
    else if (type == isStable)
      get_bool(args, &config->isStable);
    else if (type == isFOM)
      get_bool(args, &config->isFOM);
    else if (type == isMove)
      get_bool(args, &config->isMove);
    else if (type == gThreshVal)
      sscanf(args, "%f", &config->gThreshVal);
    else if (type == gThreshTime)
      sscanf(args, "%f", &config->gThreshTime);
    else if (type == aWeight)
      sscanf(args, "%f", &config->aWeight);
    else if (type == aMag)
      sscanf(args, "%f", &config->aMag);
    else if (type == aMagThresh)
      sscanf(args, "%f", &config->aMagThresh);
    else if (type == mWeight)
      sscanf(args, "%f", &config->mWeight);
    else if (type == mMag)
      sscanf(args, "%f", &config->mMag);
    else if (type == mMagThresh)
      sscanf(args, "%f", &config->mMagThresh);
    else if (type == mAng)
      sscanf(args, "%f", &config->mAng);
    else if (type == mAngThresh)
      sscanf(args, "%f", &config->mAngThresh);
    else if (type == moveAlpha)
      sscanf(args, "%f", &config->moveAlpha);
  }

  // exit function
  fclose(file);
  return 0;  
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_writeCore(
  char                  *filename,
  IMU_core_config       *config)
{
  // define internal variables
  FILE                  *file;

  // open file to contain json struct
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"isGyro\": ");      write_bool(file, config->isGyro);
  fprintf(file, "  \"isAccl\": ");      write_bool(file, config->isAccl);
  fprintf(file, "  \"isMagn\": ");      write_bool(file, config->isMagn);
  fprintf(file, "  \"isStable\": ");    write_bool(file, config->isStable);
  fprintf(file, "  \"isFOM\": ");       write_bool(file, config->isFOM);
  fprintf(file, "  \"isMove\": ");      write_bool(file, config->isMove);
  fprintf(file, "  \"gThreshVal\": %0.2f,\n",      config->gThreshVal);
  fprintf(file, "  \"gThreshTime\": %0.2f,\n",     config->gThreshTime);
  fprintf(file, "  \"aWeight\": %0.2f,\n",         config->aWeight);
  fprintf(file, "  \"aMag\": %0.2f,\n",            config->aMag);
  fprintf(file, "  \"aMagThresh\": %0.2f,\n",      config->aMagThresh);
  fprintf(file, "  \"mWeight\": %0.2f,\n",         config->mWeight);
  fprintf(file, "  \"mMag\": %0.2f,\n",            config->mMag);
  fprintf(file, "  \"mMagThresh\": %0.2f,\n",      config->mMagThresh);
  fprintf(file, "  \"mAng\": %0.2f,\n",            config->mAng);
  fprintf(file, "  \"mAngThresh\": %0.2f,\n",      config->mAngThresh);
  fprintf(file, "  \"moveAlpha\": %0.2f,\n",       config->moveAlpha);
  fprintf(file, "}\n");

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* reads calibration json file into memory (structure)
******************************************************************************/

int IMU_file_readRect( 
  char                  *filename,
  IMU_rect_config       *config)
{
  // define internal variables
  FILE                  *file;
  char                  *field;
  char                  *args;
  rect_config_enum      type;
  int                   status;

  // open json file containg config struct
  file = fopen(filename, "r");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // main loop that parse json file line by line
  while (1) {
    // read line and parse field/args
    status = IMU_file_getLine(file, &field, &args);
    if (status == 1)
      continue;
    if (status > 1 || status < 0)
      break;

    // extract arguments for the specified field 
    type = IMU_file_getField(field, rect_config_name, rect_config_size);
    if      (type == gBias) 
      get_floats(args, config->gBias, 3);
    else if (type == gMult)
      get_floats(args, config->gMult, 9);
    else if (type == aBias)
      get_floats(args, config->aBias, 3);
    else if (type == aMult)
      get_floats(args, config->aMult, 9);
    else if (type == mBias)
      get_floats(args, config->mBias, 3);
    else if (type == mMult)
      get_floats(args, config->mMult, 9);
  }

  // exit function
  fclose(file);
  return 0;   
}


/******************************************************************************
* writes calibration structure to a json file
******************************************************************************/

int IMU_file_writeRect(
  char                  *filename,
  IMU_rect_config       *config)
{
  // define internal variables
  FILE                  *file;

  // open file to contain json struct
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"gBias\": ");  write_floats(file, config->gBias, 3);
  fprintf(file, "  \"gMult\": ");  write_floats(file, config->gMult, 9);
  fprintf(file, "  \"aBias\": ");  write_floats(file, config->aBias, 3);
  fprintf(file, "  \"aMult\": ");  write_floats(file, config->aMult, 9);
  fprintf(file, "  \"mBias\": ");  write_floats(file, config->mBias, 3);
  fprintf(file, "  \"mMult\": ");  write_floats(file, config->mMult, 9);
  fprintf(file, "}\n");

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int IMU_file_readPnts(
  char                  *filename,
  IMU_pnts_config       *config)
{
  // define internal variables
  FILE*                 file;
  char*                 field;
  char*                 args;
  pnts_config_enum      type;     
  int                   status;

  // open json file containg config struct
  file = fopen(filename, "r");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // main loop that parse json file line by line
  while (1) {
    // read line and parse field/args
    status = IMU_file_getLine(file, &field, &args);
    if (status == 1)
      continue;
    if (status > 1 || status < 0)
      break;

    // extract arguments for the specified field
    type = IMU_file_getField(field, pnts_config_name, pnts_config_size);
    if      (type == pnts_isAccl)
      get_bool(args, &config->isAccl);
    else if (type == pnts_isMagn)
      get_bool(args, &config->isMagn);
    else if (type == pnts_gAlpha)
      sscanf(args, "%f", &config->gAlpha);
    else if (type == pnts_gThreshVal)
      sscanf(args, "%f", &config->gThreshVal);
    else if (type == pnts_gThreshTime)
      sscanf(args, "%f", &config->gThreshTime);
    else if (type == pnts_aAlpha)
      sscanf(args, "%f", &config->aAlpha);
    else if (type == pnts_aThresh)
      sscanf(args, "%f", &config->aThresh);
    else if (type == pnts_mAlpha)
      sscanf(args, "%f", &config->mAlpha);
    else if (type == pnts_mThresh)
      sscanf(args, "%f", &config->mThresh);
  }

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_writePnts(
  char                  *filename,
  IMU_pnts_config       *config)
{
  // define internal variables
  FILE                  *file;

  // open file to contain json struct
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"isAccl\": ");      write_bool(file, config->isAccl);
  fprintf(file, "  \"isMagn\": ");      write_bool(file, config->isMagn);
  fprintf(file, "  \"gAlpha\": %0.2f,\n",          config->gAlpha);
  fprintf(file, "  \"gThreshVal\": %0.2f,\n",      config->gThreshVal);
  fprintf(file, "  \"gThreshTime\": %0.2f,\n",     config->gThreshTime);
  fprintf(file, "  \"aAlpha\": %0.2f,\n",          config->aAlpha);
  fprintf(file, "  \"aThresh\": %0.2f,\n",         config->aThresh);
  fprintf(file, "  \"mAlpha\": %0.2f,\n",          config->mAlpha);
  fprintf(file, "  \"mThresh\": %0.2f\n",          config->mThresh);
  fprintf(file, "}\n");

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int IMU_file_readAuto(
  char                  *filename,
  IMU_auto_config       *config)
{
  // define internal variables
  FILE                  *file;
  char                  *field;
  char                  *args;
  auto_config_enum      type;     
  int                   status;

  // open json file containg config struct
  file = fopen(filename, "r");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // main loop that parse json file line by line
  while (1) {
    // read line and parse field/args
    status = IMU_file_getLine(file, &field, &args);
    if (status == 1)
      continue;
    if (status > 1 || status < 0)
      break;

    // extract arguments for the specified field
    type = IMU_file_getField(field, auto_config_name, auto_config_size);
    if      (type == auto_isGyro)
      get_bool(args, &config->isGyro);
    else if (type == auto_isAccl)
      get_bool(args, &config->isAccl);
    else if (type == auto_isMagn)
      get_bool(args, &config->isMagn);
    else if (type == auto_gAlpha)
      sscanf(args, "%f", &config->gAlpha);
    else if (type == auto_aAlpha)
      sscanf(args, "%f", &config->aAlpha);
    else if (type == auto_mAlpha)
      sscanf(args, "%f", &config->mAlpha);
  }

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_writeAuto(
  char                 *filename,
  IMU_auto_config      *config)
{
  // define internal variables
  FILE                 *file;

  // open file to contain json struct
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"isGyro\": ");      write_bool(file, config->isGyro);
  fprintf(file, "  \"isAccl\": ");      write_bool(file, config->isAccl);
  fprintf(file, "  \"isMagn\": ");      write_bool(file, config->isMagn);
  fprintf(file, "  \"gAlpha\": %0.2f,\n",          config->gAlpha);
  fprintf(file, "  \"aAlpha\": %0.2f,\n",          config->aAlpha);
  fprintf(file, "  \"mAlpha\": %0.2f\n",           config->mAlpha);
  fprintf(file, "}\n");

  // exit function
  fclose(file);
  return 0;
}

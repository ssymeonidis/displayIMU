/*
 * This file is part of quaternion-based displayIMU C/C++/QT code base
 * (https://github.com/ssymeonidis/displayIMU.git)
 * Copyright (c) 2018 Simeon Symeonidis (formerly Sensor Management Real
 * Time (SMRT) Processing Solutions)
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

// core subsystem parsing inputs
static const int   IMU_core_config_size = 19;
static const char* IMU_core_config_name[] = {
  "enable",
  "isGyro", 
  "isAccl",
  "isMagn",
  "isStable",
  "isFOM",
  "isMove",
  "isPredict",
  "gThresh",
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
  IMU_core_enable       = 0,
  IMU_core_isGyro       = 1,
  IMU_core_isAccl       = 2,
  IMU_core_isMagn       = 3,
  IMU_core_isStable     = 4,
  IMU_core_isFOM        = 5,
  IMU_core_isMove       = 6,
  IMU_core_isPredict    = 7,
  IMU_core_gThresh      = 8,
  IMU_core_gThreshTime  = 9,
  IMU_core_aWeight      = 10,
  IMU_core_aMag         = 11,
  IMU_core_aMagThresh   = 12,
  IMU_core_mWeight      = 13,
  IMU_core_mMag         = 14,
  IMU_core_mMagThresh   = 15,
  IMU_core_mAng         = 16,
  IMU_core_mAngThresh   = 17,
  IMU_core_moveAlpha    = 18
} IMU_core_config_enum;

// rect subsystem parsing inputs
static const int   IMU_rect_config_size = 7;
static const char* IMU_rect_config_name[] = {
  "enable",
  "gBias",
  "gMult",
  "aBias",
  "aMult",
  "mBias",
  "mMult"
};
typedef enum {
  IMU_rect_enable       = 0,
  IMU_rect_gBias        = 1,
  IMU_rect_gMult        = 2,
  IMU_rect_aBias        = 3,
  IMU_rect_aMult        = 4,
  IMU_rect_mBias        = 5,
  IMU_rect_mMult        = 6
} IMU_rect_config_enum;

// pnts subsystem parsing inputs
static const int   IMU_pnts_config_size = 11;
static const char* IMU_pnts_config_name[] = {
  "enable",
  "isAccl",
  "isMagn",
  "gAlpha",
  "gThresh",
  "gInitTime",
  "gHoldTime",
  "aAlpha",
  "aThresh",
  "mAlpha",
  "mThresh"
};
typedef enum {
  IMU_pnts_enable       = 0,
  IMU_pnts_isAccl       = 1,
  IMU_pnts_isMagn       = 2,
  IMU_pnts_gAlpha       = 3,
  IMU_pnts_gThresh      = 4,
  IMU_pnts_gInitTime    = 5,
  IMU_pnts_gHoldTime    = 6,
  IMU_pnts_aAlpha       = 7,
  IMU_pnts_aThresh      = 8,
  IMU_pnts_mAlpha       = 9,
  IMU_pnts_mThresh      = 10
} IMU_pnts_config_enum;

// auto subsystem parsing inputs
static const int   IMU_auto_config_size = 7;
static const char* IMU_auto_config_name[] = {
  "enable",
  "isGyro",
  "isAccl",
  "isMagn",
  "gAlpha",
  "aAlpha",
  "mAlpha"
};
typedef enum {
  IMU_auto_enable      = 0,
  IMU_auto_isGyro      = 0,
  IMU_auto_isAccl      = 1,
  IMU_auto_isMagn      = 2,
  IMU_auto_gAlpha      = 3,
  IMU_auto_gThresh     = 4,
  IMU_auto_aAlpha      = 5,
  IMU_auto_mAlpha      = 6,
} IMU_auto_config_enum;

// parsing line buffers
#define line_size 128
static char line[line_size];
static char temp[line_size];


// internal function defintions
static int  get_line   (FILE *file, char **field, char **args);
static int  get_field  (char *field, const char *names[], int size);
static int  get_floats (char *args, float *vals, int size);
static int  get_bool   (char *args, uint8_t *val);
static void write_floats (FILE *file, float *vals, int size);
static void write_bool   (FILE *file, uint8_t val);


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int IMU_file_coreLoad(
  const char*           filename,
  IMU_core_config       *config)
{
  // define internal variables
  FILE                  *file;
  char                  *field;
  char                  *args;
  IMU_core_config_enum  type;
  int                   status;

  // open configuration json file
  file = fopen(filename, "r");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // main loop that parse json file line by line
  while (1) {

    // read line and parse field/args
    status = get_line(file, &field, &args);
    if (status > 1 || status < 0)
      break;

    // extract specified field arguments  
    type = get_field(field, IMU_core_config_name, IMU_core_config_size);
    if      (type == IMU_core_enable)
      get_bool(args, &config->enable);
    else if (type == IMU_core_isGyro)
      get_bool(args, &config->isGyro);
    else if (type == IMU_core_isAccl)
      get_bool(args, &config->isAccl);
    else if (type == IMU_core_isMagn)
      get_bool(args, &config->isMagn);
    else if (type == IMU_core_isStable)
      get_bool(args, &config->isStable);
    else if (type == IMU_core_isFOM)
      get_bool(args, &config->isFOM);
    else if (type == IMU_core_isMove)
      get_bool(args, &config->isMove);
    else if (type == IMU_core_gThresh)
      sscanf(args, "%f", &config->gThresh);
    else if (type == IMU_core_gThreshTime)
      sscanf(args, "%f", &config->gThreshTime);
    else if (type == IMU_core_aWeight)
      sscanf(args, "%f", &config->aWeight);
    else if (type == IMU_core_aMag)
      sscanf(args, "%f", &config->aMag);
    else if (type == IMU_core_aMagThresh)
      sscanf(args, "%f", &config->aMagThresh);
    else if (type == IMU_core_mWeight)
      sscanf(args, "%f", &config->mWeight);
    else if (type == IMU_core_mMag)
      sscanf(args, "%f", &config->mMag);
    else if (type == IMU_core_mMagThresh)
      sscanf(args, "%f", &config->mMagThresh);
    else if (type == IMU_core_mAng)
      sscanf(args, "%f", &config->mAng);
    else if (type == IMU_core_mAngThresh)
      sscanf(args, "%f", &config->mAngThresh);
    else if (type == IMU_core_moveAlpha)
      sscanf(args, "%f", &config->moveAlpha);
  }

  // exit function
  fclose(file);
  return 0;  
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_coreSave(
  const char            *filename,
  IMU_core_config       *config)
{
  // define internal variables
  FILE                  *file;

  // open configuration json file 
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"enable\": ");      write_bool(file, config->enable);
  fprintf(file, "  \"isGyro\": ");      write_bool(file, config->isGyro);
  fprintf(file, "  \"isAccl\": ");      write_bool(file, config->isAccl);
  fprintf(file, "  \"isMagn\": ");      write_bool(file, config->isMagn);
  fprintf(file, "  \"isStable\": ");    write_bool(file, config->isStable);
  fprintf(file, "  \"isFOM\": ");       write_bool(file, config->isFOM);
  fprintf(file, "  \"isMove\": ");      write_bool(file, config->isMove);
  fprintf(file, "  \"gThresh\": %0.2f,\n",         config->gThresh);
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

int IMU_file_rectLoad( 
  const char            *filename,
  IMU_rect_config       *config)
{
  // define internal variables
  FILE                  *file;
  char                  *field;
  char                  *args;
  IMU_rect_config_enum  type;
  int                   status;

  // open configuration json file 
  file = fopen(filename, "r");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // main loop that parse json file line by line
  while (1) {
    // read line and parse field/args
    status = get_line(file, &field, &args);
    if (status == 1)
      continue;
    if (status > 1 || status < 0)
      break;

    // extract specified field arguments  
    type = get_field(field, IMU_rect_config_name, IMU_rect_config_size);
    if      (type == IMU_rect_enable) 
      get_bool(args, &config->enable);
    else if (type == IMU_rect_gBias) 
      get_floats(args, config->gBias, 3);
    else if (type == IMU_rect_gMult)
      get_floats(args, config->gMult, 9);
    else if (type == IMU_rect_aBias)
      get_floats(args, config->aBias, 3);
    else if (type == IMU_rect_aMult)
      get_floats(args, config->aMult, 9);
    else if (type == IMU_rect_mBias)
      get_floats(args, config->mBias, 3);
    else if (type == IMU_rect_mMult)
      get_floats(args, config->mMult, 9);
  }

  // exit function
  fclose(file);
  return 0;   
}


/******************************************************************************
* writes calibration structure to a json file
******************************************************************************/

int IMU_file_rectSave(
  const char            *filename,
  IMU_rect_config       *config)
{
  // define internal variables
  FILE                  *file;

  // open configuration json file 
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"enable\": ");  write_bool  (file, config->enable);
  fprintf(file, "  \"gBias\": ");   write_floats(file, config->gBias, 3);
  fprintf(file, "  \"gMult\": ");   write_floats(file, config->gMult, 9);
  fprintf(file, "  \"aBias\": ");   write_floats(file, config->aBias, 3);
  fprintf(file, "  \"aMult\": ");   write_floats(file, config->aMult, 9);
  fprintf(file, "  \"mBias\": ");   write_floats(file, config->mBias, 3);
  fprintf(file, "  \"mMult\": ");   write_floats(file, config->mMult, 9);
  fprintf(file, "}\n");

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int IMU_file_pntsLoad(
  const char            *filename,
  IMU_pnts_config       *config)
{
  // define internal variables
  FILE*                 file;
  char*                 field;
  char*                 args;
  IMU_pnts_config_enum  type;     
  int                   status;

  // open configuration json file 
  file = fopen(filename, "r");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // main loop that parse json file line by line
  while (1) {
    // read line and parse field/args
    status = get_line(file, &field, &args);
    if (status == 1)
      continue;
    if (status > 1 || status < 0)
      break;

    // extract specified field arguments  
    type = get_field(field, IMU_pnts_config_name, IMU_pnts_config_size);
    if      (type == IMU_pnts_enable)
      get_bool(args, &config->enable);
    else if (type == IMU_pnts_isAccl)
      get_bool(args, &config->isAccl);
    else if (type == IMU_pnts_isMagn)
      get_bool(args, &config->isMagn);
    else if (type == IMU_pnts_gAlpha)
      sscanf(args, "%f", &config->gAlpha);
    else if (type == IMU_pnts_gThresh)
      sscanf(args, "%f", &config->gThresh);
    else if (type == IMU_pnts_gInitTime)
      sscanf(args, "%f", &config->gInitTime);
    else if (type == IMU_pnts_gHoldTime)
      sscanf(args, "%f", &config->gHoldTime);
    else if (type == IMU_pnts_aAlpha)
      sscanf(args, "%f", &config->aAlpha);
    else if (type == IMU_pnts_aThresh)
      sscanf(args, "%f", &config->aThresh);
    else if (type == IMU_pnts_mAlpha)
      sscanf(args, "%f", &config->mAlpha);
    else if (type == IMU_pnts_mThresh)
      sscanf(args, "%f", &config->mThresh);
  }

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_pntsSave(
  const char            *filename,
  IMU_pnts_config       *config)
{
  // define internal variables
  FILE                  *file;

  // open configuration json file 
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"enable\": ");      write_bool(file, config->enable);
  fprintf(file, "  \"isAccl\": ");      write_bool(file, config->isAccl);
  fprintf(file, "  \"isMagn\": ");      write_bool(file, config->isMagn);
  fprintf(file, "  \"gAlpha\": %0.2f,\n",          config->gAlpha);
  fprintf(file, "  \"gThresh\": %0.2f,\n",         config->gThresh);
  fprintf(file, "  \"gInitTime\": %0.2f,\n",       config->gInitTime);
  fprintf(file, "  \"gHoldTime\": %0.2f,\n",       config->gHoldTime);
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

int IMU_file_autoLoad(
  const char            *filename,
  IMU_auto_config       *config)
{
  // define internal variables
  FILE                  *file;
  char                  *field;
  char                  *args;
  IMU_auto_config_enum  type;     
  int                   status;

  // open configuration json file 
  file = fopen(filename, "r");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // main loop that parse json file line by line
  while (1) {
    // read line and parse field/args
    status = get_line(file, &field, &args);
    if (status == 1)
      continue;
    if (status > 1 || status < 0)
      break;

    // extract specified field arguments  
    type = get_field(field, IMU_auto_config_name, IMU_auto_config_size);
    if      (type == IMU_auto_isGyro)
      get_bool(args, &config->isGyro);
    else if (type == IMU_auto_isAccl)
      get_bool(args, &config->isAccl);
    else if (type == IMU_auto_isMagn)
      get_bool(args, &config->isMagn);
    else if (type == IMU_auto_gAlpha)
      sscanf(args, "%f", &config->gAlpha);
    else if (type == IMU_auto_aAlpha)
      sscanf(args, "%f", &config->aAlpha);
    else if (type == IMU_auto_mAlpha)
      sscanf(args, "%f", &config->mAlpha);
  }

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_autoSave(
  const char           *filename,
  IMU_auto_config      *config)
{
  // define internal variables
  FILE                 *file;

  // open configuration json file 
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


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int IMU_file_calbLoad(
  const char           *filename, 
  IMU_calb_config      *config)
{
  return 0;
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_calbSave(
  const char           *filename, 
  IMU_calb_config      *config)
{
  return 0;
}


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int IMU_file_engnLoad(
  const char           *filename, 
  IMU_engn_config      *config)
{
  return 0;
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_engnSave(
  const char           *filename, 
  IMU_engn_config      *config)
{
  return 0;
}


/******************************************************************************
* utility function - gets a line and seperates field from arguments
******************************************************************************/

int get_line(
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

int get_field(
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

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
#include <math.h>
#include "IMU_file.h"

// core subsystem parsing inputs
static const int   IMU_core_config_size   = 17;
static const char* IMU_core_config_name[] = {
  "enable",
  "isGyro", 
  "isAccl",
  "isMagn",
  "isFOM",
  "isTran",
  "isPredict",
  "gScale",
  "aWeight",
  "aMag",
  "aMagThresh",
  "mWeight",
  "mMag",
  "mMagThresh",
  "mDot",
  "mDotThresh",
  "tranAlpha"
};
typedef enum {
  IMU_core_enable       = 0,
  IMU_core_isGyro       = 1,
  IMU_core_isAccl       = 2,
  IMU_core_isMagn       = 3,
  IMU_core_isFOM        = 4,
  IMU_core_isTran       = 5,
  IMU_core_isPredict    = 6,
  IMU_core_gScale       = 7,
  IMU_core_aWeight      = 8,
  IMU_core_aMag         = 9,
  IMU_core_aMagThresh   = 10,
  IMU_core_mWeight      = 11,
  IMU_core_mMag         = 12,
  IMU_core_mMagThresh   = 13,
  IMU_core_mDot         = 14,
  IMU_core_mDotThresh   = 15,
  IMU_core_tranAlpha    = 16
} IMU_core_config_enum;

// rect subsystem parsing inputs
static const int   IMU_rect_config_size   = 7;
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
static const int   IMU_pnts_config_size   = 12;
static const char* IMU_pnts_config_name[] = {
  "enable",
  "isGyro",
  "isAccl",
  "isMagn",
  "tHold",
  "tStable",
  "gAlpha",
  "gThresh",
  "aAlpha",
  "aThresh",
  "mAlpha",
  "mThresh"
};
typedef enum {
  IMU_pnts_enable       = 0,
  IMU_pnts_isGyro       = 1,
  IMU_pnts_isAccl       = 2,
  IMU_pnts_isMagn       = 3,
  IMU_pnts_tHold        = 4,
  IMU_pnts_tStable      = 5,
  IMU_pnts_gAlpha       = 6,
  IMU_pnts_gThresh      = 7,
  IMU_pnts_aAlpha       = 8,
  IMU_pnts_aThresh      = 9,
  IMU_pnts_mAlpha       = 10,
  IMU_pnts_mThresh      = 11
} IMU_pnts_config_enum;

// stat subsystem parsing inputs
static const int   IMU_stat_config_size   = 2;
static const char* IMU_stat_config_name[] = {
  "enable",
  "alpha"
};
typedef enum {
  IMU_stat_enable      = 0,
  IMU_stat_alpha       = 1
} IMU_stat_config_enum;

// stat subsystem parsing inputs
static const int   IMU_calb_config_size   = 1;
static const char* IMU_calb_config_name[] = {
  "enable"
};
typedef enum {
  IMU_calb_enable      = 0
} IMU_calb_config_enum;

// stat subsystem parsing inputs
static const int   IMU_engn_config_size   = 11;
static const char* IMU_engn_config_name[] = {
  "isFOM",
  "isTran",
  "isRef",
  "isAng",
  "isSensorStruct",
  "qRef",
  "configFileCore",
  "configFileRect",
  "configFilePnts",
  "configFileStat",
  "configFileCalb"
};
typedef enum {
  IMU_engn_isFOM           = 0,
  IMU_engn_isTran          = 1,
  IMU_engn_isRef           = 2,
  IMU_engn_isAng           = 3,
  IMU_engn_isSensorStruct  = 4,
  IMU_engn_qRef            = 5,
  IMU_engn_configFileCore  = 6,
  IMU_engn_configFileRect  = 7,
  IMU_engn_configFilePnts  = 8,
  IMU_engn_configFileStat  = 9,
  IMU_engn_configFileCalb  = 10
} IMU_engn_config_enum;



// parsing line buffers
#define line_size 128
static char line[line_size];
static char temp[line_size];


// internal function defintions
static int  get_line   (FILE *file, char **field, char **args);
static int  get_field  (char *field, const char *names[], int size);
static int  get_string (char *args, char *val);
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
    else if (type == IMU_core_isFOM)
      get_bool(args, &config->isFOM);
    else if (type == IMU_core_isTran)
      get_bool(args, &config->isTran);
    else if (type == IMU_core_isPredict)
      get_bool(args, &config->isPredict);
    else if (type == IMU_core_gScale)
      sscanf(args, "%f", &config->gScale);
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
    else if (type == IMU_core_mDot)
      sscanf(args, "%f", &config->mDot);
    else if (type == IMU_core_mDotThresh)
      sscanf(args, "%f", &config->mDotThresh);
    else if (type == IMU_core_tranAlpha)
      sscanf(args, "%f", &config->tranAlpha);
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
  fprintf(file, "  \"isFOM\": ");       write_bool(file, config->isFOM);
  fprintf(file, "  \"isTran\": ");      write_bool(file, config->isTran);
  fprintf(file, "  \"isPredict\": ");   write_bool(file, config->isPredict);   
  fprintf(file, "  \"gScale\": %0.6f,\n",          config->gScale);
  fprintf(file, "  \"aWeight\": %0.3f,\n",         config->aWeight);
  fprintf(file, "  \"aMag\": %0.2f,\n",            config->aMag);
  fprintf(file, "  \"aMagThresh\": %0.2f,\n",      config->aMagThresh);
  fprintf(file, "  \"mWeight\": %0.3f,\n",         config->mWeight);
  fprintf(file, "  \"mMag\": %0.2f,\n",            config->mMag);
  fprintf(file, "  \"mMagThresh\": %0.2f,\n",      config->mMagThresh);
  fprintf(file, "  \"mDot\": %0.3f,\n",            config->mDot);
  fprintf(file, "  \"mDotThresh\": %0.3f,\n",      config->mDotThresh);
  fprintf(file, "  \"tranAlpha\": %0.2f,\n",       config->tranAlpha);
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
    else if (type == IMU_pnts_isGyro)
      get_bool(args, &config->isGyro);
    else if (type == IMU_pnts_isAccl)
      get_bool(args, &config->isAccl);
    else if (type == IMU_pnts_isMagn)
      get_bool(args, &config->isMagn);
    else if (type == IMU_pnts_tHold)
      sscanf(args, "%d", &config->tHold);
    else if (type == IMU_pnts_tStable)
      sscanf(args, "%d", &config->tStable);
    else if (type == IMU_pnts_gAlpha)
      sscanf(args, "%f", &config->gAlpha);
    else if (type == IMU_pnts_gThresh)
      sscanf(args, "%f", &config->gThresh);
    else if (type == IMU_pnts_aAlpha)
      sscanf(args, "%f", &config->aAlpha);
    else if (type == IMU_pnts_aThresh)
      sscanf(args, "%f", &config->aThresh);
    else if (type == IMU_pnts_mAlpha)
      sscanf(args, "%f", &config->mAlpha);
    else if (type == IMU_pnts_mThresh)
      sscanf(args, "%f", &config->mThresh);
  }

  // scale parameters
  config->tHold    = (uint32_t)(config->tHold   * 100.0f);
  config->tStable  = (uint32_t)(config->tStable * 100.0f);
  config->gThresh  = config->gThresh * config->gThresh;
  config->aThresh  = config->aThresh * config->aThresh;
  config->mThresh  = config->mThresh * config->mThresh;

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
  fprintf(file, "  \"enable\": ");    write_bool(file, config->enable);
  fprintf(file, "  \"isGyro\": ");    write_bool(file, config->isGyro);
  fprintf(file, "  \"isAccl\": ");    write_bool(file, config->isAccl);
  fprintf(file, "  \"isMagn\": ");    write_bool(file, config->isMagn);
  fprintf(file, "  \"tHold\": %d,\n",                  config->tHold   / 100);
  fprintf(file, "  \"tStable\": %d,\n",                config->tStable / 100);
  fprintf(file, "  \"gAlpha\": %0.3f,\n",              config->gAlpha);
  fprintf(file, "  \"gThresh\": %0.2f,\n",        sqrt(config->gThresh));
  fprintf(file, "  \"aAlpha\": %0.3f,\n",              config->aAlpha);
  fprintf(file, "  \"aThresh\": %0.2f,\n",        sqrt(config->aThresh));
  fprintf(file, "  \"mAlpha\": %0.3f,\n",              config->mAlpha);
  fprintf(file, "  \"mThresh\": %0.2f\n",         sqrt(config->mThresh));
  fprintf(file, "}\n");

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int IMU_file_statLoad(
  const char            *filename,
  IMU_stat_config       *config)
{
  // define internal variables
  FILE                  *file;
  char                  *field;
  char                  *args;
  IMU_stat_config_enum  type;     
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
    type = get_field(field, IMU_stat_config_name, IMU_stat_config_size);
    if      (type == IMU_stat_enable)
      get_bool(args, &config->enable);
    else if (type == IMU_stat_alpha)
      sscanf(args, "%f", &config->alpha);
  }

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_statSave(
  const char           *filename,
  IMU_stat_config      *config)
{
  // define internal variables
  FILE                 *file;

  // open configuration json file 
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"enable\": ");         write_bool(file, config->enable);
  fprintf(file, "  \"alpha\": %0.5f,\n",   config->alpha);
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
  // define internal variables
  FILE                  *file;
  char                  *field;
  char                  *args;
  IMU_calb_config_enum  type;     
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
    type = get_field(field, IMU_calb_config_name, IMU_calb_config_size);
    if      (type == IMU_calb_enable)
      get_bool(args, &config->enable);
  }

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_calbSave(
  const char           *filename, 
  IMU_calb_config      *config)
{
  // define internal variables
  FILE                 *file;

  // open configuration json file 
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  fprintf(file, "{\n");
  fprintf(file, "  \"enable\": ");       write_bool(file, config->enable);
  fprintf(file, "}\n");

  // exit function
  fclose(file);
  return 0;
}


/******************************************************************************
* reads configuration json file into memory (structure)
******************************************************************************/

int IMU_file_engnLoad(
  const char            *filename, 
  IMU_engn_config       *config)
{
  // define internal variables
  FILE                  *file;
  char                  *field;
  char                  *args;
  IMU_engn_config_enum  type;
  int                   status;

  // open configuration json file
  file = fopen(filename, "r");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // initialize filename to be empty
  config->configFileCore[0] = '\0';
  config->configFileRect[0] = '\0';
  config->configFilePnts[0] = '\0';
  config->configFileStat[0] = '\0';
  config->configFileCalb[0] = '\0';

  // main loop that parse json file line by line
  while (1) {

    // read line and parse field/args
    status = get_line(file, &field, &args);
    if (status > 1 || status < 0)
      break;

    // extract specified field arguments  
    type = get_field(field, IMU_engn_config_name, IMU_engn_config_size);
    if      (type == IMU_engn_isFOM)
      get_bool(args, &config->isFOM);
    else if (type == IMU_engn_isTran)
      get_bool(args, &config->isTran);
    else if (type == IMU_engn_isRef)
      get_bool(args, &config->isRef);
    else if (type == IMU_engn_isAng)
      get_bool(args, &config->isAng);
    else if (type == IMU_engn_isSensorStruct)
      get_bool(args, &config->isSensorStruct);
    else if (type == IMU_engn_qRef) 
      get_floats(args, config->qRef, 4);
    else if (type == IMU_engn_configFileCore)
      get_string(args, config->configFileCore);
    else if (type == IMU_engn_configFileRect)
      get_string(args, config->configFileRect);
    else if (type == IMU_engn_configFilePnts)
      get_string(args, config->configFilePnts);
    else if (type == IMU_engn_configFileStat)
      get_string(args, config->configFileStat);
    else if (type == IMU_engn_configFileCalb)
      get_string(args, config->configFileCalb);
  }

  // exit function
  fclose(file);
  return 0;  
}


/******************************************************************************
* writes configuration structure to a json file
******************************************************************************/

int IMU_file_engnSave(
  const char            *filename, 
  IMU_engn_config       *config)
{
  // define internal variables
  FILE                  *file;

  // open configuration json file 
  file = fopen(filename, "w");
  if (file == NULL)
    return IMU_FILE_INVALID_FILE;

  // write contents to json file one line at a time
  uint8_t               isSensor = config->isSensorStruct;
  fprintf(file, "{\n");
  fprintf(file, "  \"isFOM\": ");           write_bool  (file, config->isFOM);
  fprintf(file, "  \"isTran\": ");          write_bool  (file, config->isTran);
  fprintf(file, "  \"isRef\": ");           write_bool  (file, config->isRef);
  fprintf(file, "  \"isAng\": ");           write_bool  (file, config->isAng);
  fprintf(file, "  \"isSensorStruct\": ");  write_bool  (file, isSensor);
  fprintf(file, "  \"qRef\": ");            write_floats(file, config->qRef, 4);
  if (config->configFileCore[0] != '\0')
    fprintf(file, "  \"configFileCore\": %s", config->configFileCore);
  if (config->configFileRect[0] != '\0')
    fprintf(file, "  \"configFileRect\": %s", config->configFileRect);
  if (config->configFilePnts[0] != '\0')
    fprintf(file, "  \"configFilePnts\": %s", config->configFilePnts);
  if (config->configFileStat[0] != '\0')
    fprintf(file, "  \"configFileStat\": %s", config->configFileStat);
  if (config->configFileCalb[0] != '\0')
    fprintf(file, "  \"configFileCalb\": %s", config->configFileCalb);
  fprintf(file, "}\n");

  // exit function
  fclose(file);
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
* utility function - gets a line and seperates field from arguments
******************************************************************************/

int get_string(
  char                 *args, 
  char                 *val)
{
  char *tmp = strtok(args, "\" ");
  strcpy(val, tmp);
  return 0;
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
    fprintf(file, "%0.5f, ", vals[i]);
  fprintf(file, "%0.5f],\n", vals[size-1]); 
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

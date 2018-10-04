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

#ifndef _DATA_PARSE_H
#define _DATA_PARSE_H

// include statements
#include "IMU_correct.h"
#include "IMU_core.h"
#include "IMU_calib_pnts.h"
#include "IMU_calib_auto.h"
#include "IMU_calib_ctrl.h"


// define IMU state structure
struct dataParse_state {
  unsigned short                 id_correct;
  unsigned short                 id_core;
  unsigned short                 id_calib_pnts;
  unsigned short                 id_calib_auto;
  unsigned short                 id_calib_ctrl;
  struct IMU_correct_config      *config_correct;
  struct IMU_core_config         *config_core;
  struct IMU_calib_pnts_config   *config_calib_pnts;
  struct IMU_calib_auto_config   *config_calib_auto;
};

// define input sensor data structure
struct dataParse_sensor {
  float                          gyroRaw[3];
  float                          gyroCor[3];
  float                          gyroFltr[3];
  float                          acclRaw[3];
  float                          acclCor[3];
  float                          acclFltr[3];
  float                          magnRaw[3];
  float                          magnCor[3];
  float                          magnFltr[3];
  float                          lastTime;
}; 

// define IMU estimate data structure
struct dataParse_estim {
  float                          ang[3];
  float                          move[3];  
  IMU_calib_pnts_entry           *pnt;
  IMU_core_FOM                   FOMcore[3];
  IMU_calib_ctrl_FOM             FOMcalib;
};


// define the sensor data structure
extern dataParse_sensor  sensor;
extern dataParse_estim   estim;
extern dataParse_state   stateIMU;

// initialization function
void data_init(
  char*                  file_correct,
  char*                  file_core,
  char*                  file_calib_pnts,
  char*                  file_calib_auto);

// access functions
void data_start_log      (const char* filename);
void data_start_UDP      (int portno);
void data_start_CSV      (const char* filename);
void data_close          ();
void data_process_datum  ();
void *data_run           (void* id);

#endif

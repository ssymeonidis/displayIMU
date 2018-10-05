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
#include <stdint.h>
#include "IMU_rect.h"
#include "IMU_core.h"
#include "IMU_pnts.h"
#include "IMU_auto.h"
#include "IMU_calb.h"


// define IMU state structure
typedef struct {
  float                  q_ref[4];
  uint8_t                exit_thread;
} dataParse_ctrl;

typedef struct {
  uint16_t               idRect;
  uint16_t               idCore;
  uint16_t               idPnts;
  uint16_t               idAuto;
  uint16_t               idCalb;
  IMU_rect_config        *configRect;
  IMU_core_config        *configCore;
  IMU_pnts_config        *configPnts;
  IMU_auto_config        *configAuto;
} dataParse_state;

// define input sensor data structure
typedef struct {
  float                  gyroRaw[3];
  float                  gyroCor[3];
  float                  gyroFltr[3];
  float                  acclRaw[3];
  float                  acclCor[3];
  float                  acclFltr[3];
  float                  magnRaw[3];
  float                  magnCor[3];
  float                  magnFltr[3];
  float                  time;
} dataParse_sensor; 

// define IMU estimate data structure
typedef struct {
  float                  *q_org;
  float                  q[4];
  float                  ang[3];
  float                  move[3];  
  IMU_pnts_entry         *pnt;
  IMU_core_FOM           FOMcore[3];
  IMU_calb_FOM           FOMcalib;
} dataParse_estim;


// define the sensor data structure
extern dataParse_ctrl    ctrl;
extern dataParse_sensor  sensor;
extern dataParse_estim   estim;
extern dataParse_state   state;

// initialization function
void data_init(char* file_rect, char* file_core, 
               char* file_pnts, char* file_calb);

// access functions
void data_start_log      (const char* filename);
void data_start_UDP      (int portno);
void data_start_CSV      (const char* filename);
void data_close          ();
void data_process_datum  ();
void *data_run           (void* id);

#endif

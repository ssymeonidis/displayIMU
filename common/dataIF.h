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

#ifndef _DATA_IF_H
#define _DATA_IF_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>
#include "IMU_rect.h"
#include "IMU_core.h"
#include "IMU_pnts.h"
#include "IMU_auto.h"
#include "IMU_calb.h"


// define IMU state structure
typedef struct {
  uint8_t                isRealtime;
  uint8_t                exitThread;
  uint8_t                isExit;
} dataIF_ctrl;

typedef struct {
  uint8_t                isCSV;
  uint8_t                isFirstFrame;
  double                 timeInitSys;
  double                 timeInitSen;
  uint16_t               idRect;
  uint16_t               idCore;
  uint16_t               idPnts;
  uint16_t               idAuto;
  uint16_t               idCalb;
  IMU_pnts_entry         *pnt;
  IMU_FOM_core           FOM[3];
} dataIF_state;

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
} dataIF_sensor; 


// initialization function
void dataIF_init        (uint16_t idRect, uint16_t idCore, 
                         uint16_t idPnts, uint16_t idAuto);
void dataIF_getSensor   (dataIF_sensor **sensor);

// access functions
void dataIF_startUDP    (int         portno);
void dataIF_startCSV    (const char* filename);
void dataIF_process     ();
void dataIF_exit        ();
void *dataIF_run        (void* id);


#ifdef __cplusplus
}
#endif

#endif

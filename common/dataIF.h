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

#ifndef _DATA_IF_H
#define _DATA_IF_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>
#include "IMU_engn.h"


// define IMU state structure
typedef struct {
  uint8_t                isRealtime;
  uint8_t                isRepeat;
} dataIF_config;

typedef struct {
  uint16_t               imuID;
  IMU_engn_config        *imuConfig;
  uint8_t                isCSV;
  uint8_t                isFirstFrame;
  double                 timeInitSys;
  double                 timeInitSen;
  uint8_t                exitThread;
  uint8_t                isExit;
} dataIF_state;


// initialization function
uint16_t  dataIF_init        (IMU_engn_type);

// access functions
void      dataIF_startUDP    (int         portno);
void      dataIF_startCSV    (const char* filename);
int       dataIF_process     ();
void      dataIF_exit        ();
void      *dataIF_run        (void* id);


#ifdef __cplusplus
}
#endif

#endif

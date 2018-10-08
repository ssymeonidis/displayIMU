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

#ifndef _IMU_IF_H
#define _IMU_IF_H

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
  float                  t_ms_frame;
  float                  q_ref[4];
  uint8_t                isDataLog;
  uint8_t                exitThread;
  uint8_t                isExit;
} imuIF_ctrl;

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
} imuIF_state;

// define IMU estimate data structure
typedef struct {
  float                  q_org[4];
  float                  q[4];
  float                  ang[3];
  float                  move[3];
  IMU_calb_FOM           FOMcalib;
} imuIF_estm;


// initialization function
void imuIF_init          (char* file_rect, char* file_core, 
                          char* file_pnts, char* file_calb);
void imuIF_getState      (imuIF_state**);
void imuIF_getPntr       (imuIF_estm**);
void imuIF_startLog      (char* filename);

// access functions
void imuIF_reset         ();
void imuIF_setRef        (float* ref);
void imuIF_setRefCur     ();
void imuIF_startCalb     ();
void imuIF_getEstm       (imuIF_estm*);
void imuIF_exit          ();
void *imuIF_run           (void* id);

#ifdef __cplusplus
}
#endif

#endif

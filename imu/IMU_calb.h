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

#ifndef _IMU_CALB_H
#define _IMU_CALB_H

#include "IMU_rect.h"
#include "IMU_core.h"
#include "IMU_pnts.h"
#include "IMU_auto.h"
#include "IMU_type.h"

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>
#include "IMU_type.h"

// define status codes
#define IMU_CALB_UPDATED         1

// define error codes
#define IMU_CALB_INST_OVERFLOW  -1
#define IMU_CALB_BAD_INST       -2
#define IMU_CALB_BAD_MODE       -3


// define calibration types
typedef enum {
  IMU_calb_NA               = -1,
  IMU_calb_4pnt             = 0,
  IMU_calb_6pnt             = 1
} IMU_calb_mode;

typedef struct  {
  uint8_t                   empty;
} IMU_calb_config;

// define internal state
typedef struct  {
  IMU_calb_mode             mode;
  IMU_rect_config           rect;
  IMU_rect_config           rect_org;
  IMU_core_config           core;
  IMU_core_config           core_org;
  IMU_calb_FOM              FOM;
  uint16_t                  numPnts;
} IMU_calb_state;


// control side functions 
int IMU_calb_init     (uint16_t *id, IMU_calb_config**);
int IMU_calb_getConfig (uint16_t id, IMU_calb_config**);
int IMU_calb_setStruct (uint16_t id, IMU_rect_config*, IMU_core_config*);

// system access function
int IMU_calb_reset     (uint16_t id);
int IMU_calb_start     (uint16_t id, IMU_calb_mode);
int IMU_calb_status    (uint16_t id, IMU_calb_FOM**);
int IMU_calb_save      (uint16_t id, IMU_rect_config*, IMU_core_config*);

// sensor interface functions
int IMU_calb_pnts      (uint16_t id, IMU_pnts_entry*); 
int IMU_calb_auto      (uint16_t id, IMU_auto_state*);


#ifdef __cplusplus
}
#endif

#endif

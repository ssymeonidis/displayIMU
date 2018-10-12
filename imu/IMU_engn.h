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

#ifndef _IMU_ENGN_H
#define _IMU_ENGN_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#include <stdint.h>
#include "IMU_type.h"
#include "IMU_core.h"
#include "IMU_rect.h"
#include "IMU_pnts.h"
#include "IMU_auto.h"
#include "IMU_calb.h"

// define status codes
#define IMU_ENGN_CALBFNC_SAVED           1
#define IMU_ENGN_CALBFNC_REJECTED        2

// define error codes
#define IMU_ENGN_INST_OVERFLOW           -1
#define IMU_ENGN_BAD_ENGN_TYPE           -2
#define IMU_ENGN_FAILED_SYS_INIT         -3
#define IMU_ENGN_FAILED_RESET            -4
#define IMU_ENGN_BAD_INST                -5
#define IMU_ENGN_UNINITIALIZE_SYS        -6
#define IMU_ENGN_NONEXISTANT_SYSID       -7
#define IMU_ENGN_NONEXISTANT_STRUCT      -8
#define IMU_ENGN_DISABLED_SENSOR_STRUCT  -9
#define IMU_ENGN_SUBSYSTEM_FAILURE       -10
#define IMU_ENGN_SENSOR_STRUCT_COPY_FAIL -11
#define IMU_ENGN_FAILED_THREAD           -12
#define IMU_ENGN_FAILED_MUTEX            -13
#define IMU_ENGN_QUEUE_OVERFLOW          -14
#define IMU_ENGN_BAD_PNTR                -15


// define the configuration structure (values tuned for a part)
typedef struct {
  uint8_t               isRect;
  uint8_t               isPnts;
  uint8_t               isAuto;
  uint8_t               isCalb;
  uint8_t               isEstmAccl;
  uint8_t               isQuatOnly;
  uint8_t               isFOM;
  uint8_t               isSensorStruct;
  float                 threshFOM;
} IMU_engn_config;

typedef struct {
  uint16_t              exitThread;
  uint16_t              isExit;
  uint16_t              idCore;
  uint16_t              idRect;
  uint16_t              idPnts;
  uint16_t              idAuto;
  uint16_t              idCalb;
  IMU_core_config       *configCore;
  IMU_rect_config       *configRect;
  IMU_pnts_config       *configPnts;
  IMU_auto_config       *configAuto;
  IMU_calb_config       *configCalb;
  float                 q_ref[4];
  int                   (*fncCalb)(uint16_t, IMU_calb_FOM*);    // test only
} IMU_engn_state;

// define the configuration structure
typedef union {
  IMU_core_config       *configCore;
  IMU_rect_config       *configRect;
  IMU_pnts_config       *configPnts;
  IMU_auto_config       *configAuto;
  IMU_calb_config       *configCalb;
  IMU_engn_config       *configEngn;
} IMU_union_config;

// define the state structure 
typedef union {
  IMU_core_state        *stateCore;
  IMU_pnts_state        *statePnts;
  IMU_auto_state        *stateAuto;
  IMU_calb_state        *stateCalb;
  IMU_engn_state        *stateEngn;
} IMU_union_state;

typedef enum {
  IMU_engn_core_only    = 0,
  IMU_engn_rect_core    = 1,
  IMU_engn_calb_pnts    = 2,
  IMU_engn_calb_auto    = 3,
  IMU_engn_calb_full    = 4
} IMU_engn_type;

typedef enum {
  IMU_engn_core         = 0,
  IMU_engn_rect         = 1,
  IMU_engn_pnts         = 2,
  IMU_engn_auto         = 3,
  IMU_engn_calb         = 4,
  IMU_engn_self         = 5
} IMU_engn_system;

typedef struct {
  float                 time;
  IMU_TYPE              gRaw[3];
  IMU_TYPE              gCor[3];
  float                 gFlt[3];
  IMU_core_FOM_gyro     gFOM;
  IMU_TYPE              aRaw[3];
  IMU_TYPE              aCor[3];
  float                 aFlt[3];
  IMU_core_FOM_accl     aFOM;
  IMU_TYPE              mRaw[3];
  IMU_TYPE              mCor[3];
  float                 mFlt[3];
  IMU_core_FOM_magn     mFOM;
} IMU_engn_sensor;

// define IMU estimate data structure
typedef struct {
  float                 q_org[4];
  float                 q[4];
  float                 ang[3];
  float                 move[3];
} IMU_engn_estm;

#if IMU_ENGN_QUEUE_SIZE
typedef struct {
  uint16_t              id    [IMU_ENGN_QUEUE_SIZE+1];
  IMU_datum             datum [IMU_ENGN_QUEUE_SIZE+1];
  int                   first;
  int                   last;
  int                   count;
} IMU_engn_queue;
#endif


// data structure access functions
int IMU_engn_init       (IMU_engn_type, uint16_t *id, IMU_engn_config**);
int IMU_engn_getSysID   (uint16_t id, IMU_engn_system, uint16_t *sysID);
int IMU_engn_getConfig  (uint16_t id, IMU_engn_system, IMU_union_config*);
int IMU_engn_getState   (uint16_t id, IMU_engn_system, IMU_union_state*);
int IMU_engn_getSensor  (uint16_t id, IMU_engn_sensor**);
int IMU_engn_setCalbFnc (uint16_t id, int (*fncCalb)(uint16_t, IMU_calb_FOM*));

// enable/disable controls for queue
#if IMU_ENGN_QUEUE_SIZE
int IMU_engn_start      ();
int IMU_engn_stop       ();
#endif

// functions to read/write config structures
int IMU_engn_load       (uint16_t id, const char* filename, IMU_engn_system);
int IMU_engn_save       (uint16_t id, const char* filename, IMU_engn_system);

// 
int IMU_engn_reset      (uint16_t id);
int IMU_engn_setRef     (uint16_t id, float* ref);
int IMU_engn_setRefCur  (uint16_t id);
int IMU_engn_calbStart  (uint16_t id, IMU_calb_mode);
int IMU_engn_calbSave   (uint16_t id);

// 
int IMU_engn_datum      (uint16_t id, IMU_datum*);
int IMU_engn_data3      (uint16_t id, IMU_data3*);
int IMU_engn_getEstm    (uint16_t id, float t, IMU_engn_estm*);


#ifdef __cplusplus
}
#endif

#endif

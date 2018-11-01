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
#include "IMU_stat.h"
#include "IMU_calb.h"

// define error codes
#define IMU_ENGN_INST_OVERFLOW           -1
#define IMU_ENGN_BAD_INST                -2
#define IMU_ENGN_BAD_ENGN_TYPE           -3
#define IMU_ENGN_SUBSYSTEM_FAILURE       -4
#define IMU_ENGN_UNINITIALIZE_SYS        -5
#define IMU_ENGN_NONEXISTANT_SYSID       -6
#define IMU_ENGN_NONEXISTANT_STRUCT      -7
#define IMU_ENGN_DISABLED_SENSOR_STRUCT  -8
#define IMU_ENGN_SENSOR_STRUCT_COPY_FAIL -9
#define IMU_ENGN_FAILED_THREAD           -10
#define IMU_ENGN_FAILED_MUTEX            -11
#define IMU_ENGN_QUEUE_OVERFLOW          -12


// configuration structure definition
typedef struct {
  uint8_t               isRect;          // enable rectify subsystem
  uint8_t               isPnts;          // enable stable point collection
  uint8_t               isStat;          // enable continous metric collection
  uint8_t               isCalb;          // enable calibration subsystem 
  uint8_t               isEstmAccl;      // enable accl estm (minus gravity) 
  uint8_t               isQuatOnly;      // disable conversion to Euler angles
  uint8_t               isFOM;           // disable calculation of FOMs
  uint8_t               isSensorStruct;  // enable storage of sensor data
  float                 q_ref[4];        // quaternion reference
} IMU_engn_config;

// system state structure definition
typedef struct {
  uint16_t              idCore;          // core subsystem id
  uint16_t              idRect;          // rect subsystem id
  uint16_t              idPnts;          // pnts subsystem id
  uint16_t              idStat;          // stat subsystem id
  uint16_t              idCalb;          // calb subsystem id
  IMU_core_config       *configCore;     // core configuration pointer
  IMU_rect_config       *configRect;     // rect configuration pointer
  IMU_pnts_config       *configPnts;     // pnts configuration pointer
  IMU_stat_config       *configStat;     // stat configuration pointer
  IMU_calb_config       *configCalb;     // calb configuration pointer
  int                   core;            // status of IMU core
  int                   quat;            // status of IMU estm
  int                   tran;            // status of IMU tran
  int                   rect;            // status of IMU rect
  int                   pnts;            // status of IMU pnts
  int                   stat;            // status of IMU stat
  int                   calb;            // status of IMU calb
  uint8_t               isExit;          // commands thread to exit
  uint8_t               exitThread;      // confirms thread has exited 
} IMU_engn_state;

// define which subsystems are running
typedef enum {                           // input to IMU_engn_init
  IMU_engn_core_only    = 0,             // core subystem only
  IMU_engn_rect_core    = 1,             // rect and core
  IMU_engn_calb_pnts    = 2,             // rect, pnts, calb, and core
  IMU_engn_calb_stat    = 3,             // rect, stat, calb, and core
  IMU_engn_calb_full    = 4              // all susbsystems running
} IMU_engn_type;

// input to multiple functions
typedef enum {
  IMU_engn_core         = 0,
  IMU_engn_rect         = 1,
  IMU_engn_pnts         = 2,
  IMU_engn_stat         = 3,
  IMU_engn_calb         = 4,
  IMU_engn_self         = 5
} IMU_engn_system;

// input to IMU_engn_getConfig
typedef union {
  IMU_core_config       *core;
  IMU_rect_config       *rect;
  IMU_pnts_config       *pnts;
  IMU_stat_config       *stat;
  IMU_calb_config       *calb;
  IMU_engn_config       *engn;
} IMU_union_config;

// input to IMU_engn_getState
typedef union {
  IMU_core_state        *core;
  IMU_pnts_state        *pnts;
  IMU_stat_state        *stat;
  IMU_calb_state        *calb;
  IMU_engn_state        *engn;
} IMU_union_state;

// sensor data structure 
typedef struct {
  uint32_t              time;
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

// estimate data structure
typedef struct {
  float                 q_org[4];
  float                 q[4];
  float                 ang[3];
  float                 pos[3];
} IMU_engn_estm;


// data structure access functions
int IMU_engn_init         (IMU_engn_type, uint16_t *id);
int IMU_engn_getSysID     (uint16_t id, IMU_engn_system, uint16_t *sysID);
int IMU_engn_getConfig    (uint16_t id, IMU_engn_system, IMU_union_config*);
int IMU_engn_getState     (uint16_t id, IMU_engn_system, IMU_union_state*);
int IMU_engn_getSensor    (uint16_t id, IMU_engn_sensor**);
int IMU_engn_setCalbFnc   (uint16_t id, void (*fnc)(IMU_CALB_FNC_ARG), void*);
int IMU_engn_setStableFnc (uint16_t id, void (*fnc)(IMU_PNTS_FNC_ARG), void*);
int IMU_engn_setBreakFnc  (uint16_t id, void (*fnc)(IMU_PNTS_FNC_ARG), void*);

// enable/disable queue controls
int IMU_engn_start        ();
int IMU_engn_stop         ();

// read/write config fimctopms
int IMU_engn_load         (uint16_t id, const char* filename, IMU_engn_system);
int IMU_engn_save         (uint16_t id, const char* filename, IMU_engn_system);

// system control functions
int IMU_engn_reset        (uint16_t id);
int IMU_engn_setRef       (uint16_t id, float* ref);
int IMU_engn_setRefCur    (uint16_t id);
int IMU_engn_calbStart    (uint16_t id, IMU_calb_mode);
int IMU_engn_calbSave     (uint16_t id);

// state update/estimation functions
int IMU_engn_datum        (uint16_t id, IMU_datum*);
int IMU_engn_data3        (uint16_t id, IMU_data3*);
int IMU_engn_getEstm      (uint16_t id, float t, IMU_engn_estm*);


#ifdef __cplusplus
}
#endif

#endif

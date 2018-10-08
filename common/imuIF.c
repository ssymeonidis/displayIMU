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

// include statements 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/time.h>
#include <errno.h>
#include "IMU_file.h"
#include "IMU_math.h"
#include "imuIF.h"


// define the sensor data structure
imuIF_ctrl               ctrl;
imuIF_estm               estm;
imuIF_state              state;

// define internal/external variables
FILE*                    log_file;


/******************************************************************************
* constructor for dataParse functions
******************************************************************************/

void imuIF_init(
  char                   *file_rect,
  char                   *file_core,
  char                   *file_pnts,
  char                   *file_auto)
{
  // initialize and get handles to all IMU functions
  IMU_rect_init  (&state.idRect,  &state.configRect);
  IMU_core_init  (&state.idCore,  &state.configCore);
  IMU_pnts_init  (&state.idPnts,  &state.configPnts);
  IMU_auto_init  (&state.idAuto,  &state.configAuto);
  IMU_calb_init  (&state.idCalb);

  // read config structures from json files
  if (!file_rect) IMU_file_rectOpen  (file_rect, state.configRect);
  if (!file_core) IMU_file_coreOpen  (file_core, state.configCore);
  if (!file_pnts) IMU_file_pntsOpen  (file_pnts, state.configPnts);
  if (!file_auto) IMU_file_autoOpen  (file_auto, state.configAuto);

  // initialize the reference vector
  ctrl.q_ref[0]          = 1;
  ctrl.q_ref[1]          = 0;
  ctrl.q_ref[2]          = 0;
  ctrl.q_ref[3]          = 0;
  
  // initialize the control structure
  ctrl.isDataLog         = 0;
}


/******************************************************************************
* get pointer to sensor structure
******************************************************************************/

void imuIF_getState(
  imuIF_state            **pntr)
{
  *pntr                  = &state;
}


/******************************************************************************
* get pointer to sensor structure
******************************************************************************/

void imuIF_getPntr(
  imuIF_estm             **pntr)
{
  *pntr                  = &estm;
}


/******************************************************************************
* used to start logging results to csv file
******************************************************************************/

void data_start_log(
  const char             *filename)
{
  log_file    = fopen(filename, "w+");
  if (!log_file) {
    perror("ERROR opening log file"); 
    exit(1);
  }
  setbuf(log_file, NULL);
  ctrl.isDataLog = 1;
}


/******************************************************************************
* resets major IMU components
******************************************************************************/

void imuIF_reset()
{
  IMU_core_reset(state.idCore);
  IMU_pnts_stop(state.idCore);
  IMU_auto_reset(state.idCore);
}


/******************************************************************************
* main function for receiving/parsing data and updating IMU
******************************************************************************/

void imuIF_setRef(
  float*                 ref)
{
  // get estimates
  ctrl.q_ref[0]          = ref[0];
  ctrl.q_ref[1]          = ref[1];
  ctrl.q_ref[2]          = ref[2];
  ctrl.q_ref[3]          = ref[3];
}


/******************************************************************************
* main function for receiving/parsing data and updating IMU
******************************************************************************/

void imuIF_setRefCur()
{
  // get estimates
  ctrl.q_ref[0]          =  estm.q_org[0];
  ctrl.q_ref[1]          = -estm.q_org[1];
  ctrl.q_ref[2]          = -estm.q_org[2];
  ctrl.q_ref[3]          = -estm.q_org[3];
}


/******************************************************************************
* initiates a manual calibration
******************************************************************************/

void imuIF_startCalb()
{
}


/******************************************************************************
* main function for estimating current state
******************************************************************************/

void imuIF_udpate()
{
  // get estimates
  IMU_core_estmQuat(state.idCore, estm.q_org);
  IMU_core_estmAccl(state.idCore, estm.move);
  IMU_math_applyRef(estm.q_org, ctrl.q_ref, estm.q);
  IMU_math_quatToEuler(estm.q_org, estm.ang);
  
  // write state to log file
  if (ctrl.isDataLog) {
    fprintf(log_file, "%f, %f, %f, %f, %f, %f, %f, %f, ",
      estm.q_org[0], estm.q_org[1], estm.q_org[2], estm.q_org[3],
      estm.q[0],     estm.q[1],     estm.q[2],     estm.q[3]);
    fprintf(log_file, "%f, %f, %f, %f, %f, %f\n",
      estm.ang[0],   estm.ang[1],   estm.ang[2],
      estm.move[0],  estm.move[1],  estm.move[2]);
  }
}


/******************************************************************************
* main function for estimating current state
******************************************************************************/

void imuIF_getEstm(
  imuIF_estm             *pntr)
{
  imuIF_udpate();
  memcpy(pntr, estm.q_org, sizeof(imuIF_estm));
}


/******************************************************************************
* deconstructor for overall data parse block
******************************************************************************/

void imuIF_exit()
{
  ctrl.exitThread = 1;
  while(!ctrl.isExit)
    usleep(100); 
  if (!ctrl.isDataLog)
    fclose(log_file);
}


/******************************************************************************
* "continous run" function handle for recreating a thread 
******************************************************************************/

void* imuIF_run(void* id)
{
  // define local variables
  struct timeval         time;
  double                 time_init;
  double                 time_cur;
  double                 time_delta;

  // main processing loop
  ctrl.exitThread = 0;
  ctrl.isExit     = 0;
  while(!ctrl.exitThread) {
    gettimeofday(&time, NULL);
    time_init            = time.tv_sec * 1000000 + time.tv_usec;
    imuIF_udpate();
    while (1) {
      gettimeofday(&time, NULL);
      time_cur           = time.tv_sec * 1000000 + time.tv_usec;
      time_delta         = (time_cur - time_init) / 1000;
      if (time_delta > ctrl.t_ms_frame) 
        usleep(time_delta); 
      else
        break;
    }
  }
  ctrl.isExit     = 1;
  return 0;
}

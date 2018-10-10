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

#ifndef _IMU_THRD_H
#define _IMU_THRD_H

#ifdef __cplusplus
extern "C" {
#endif

// include statements
#if IMU_USE_PTHREAD
#include <pthread.h>
#endif


/******************************************************************************
* used to create mutex for managing critical sections 
******************************************************************************/

inline int IMU_thrd_mutex_init(
  #if IMU_USE_PTHREAD
  pthread_mutex_t       *lock)
  #endif
{
  #if IMU_USE_PTHREAD
  return pthread_mutex_init(lock, NULL);
  #else
  return 0;
  #endif
}


/******************************************************************************
* used prior to entering critical section 
******************************************************************************/

inline void IMU_thrd_mutex_lock(
  #if IMU_USE_PTHREAD
  pthread_mutex_t       *lock)
  #endif
{
  #if IMU_USE_PTHREAD
  pthread_mutex_lock(lock);
  #endif
}


/******************************************************************************
* utility function - used prior to leaving critical section
******************************************************************************/

inline void IMU_thrd_mutex_unlock(
  #if IMU_USE_PTHREAD
  pthread_mutex_t       *lock)
  #endif
{
  #if IMU_USE_PTHREAD
  pthread_mutex_unlock(lock);
  #endif
}


#endif

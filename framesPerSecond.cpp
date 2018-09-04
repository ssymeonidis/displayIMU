/*
 * This file is part of quaternion-based displayIMU C++/QT code base
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

#include "framesPerSecond.h"
#include <stdio.h>


framesPerSecond::framesPerSecond()
{
  captureRate         = 2;
  fps                 = 0.0;
  ticks               = 0;
  time(&rawtime);
  timeinfo            = localtime(&rawtime);
  captureLast         = timeinfo->tm_sec;
}


float framesPerSecond::get()
{
  ticks++;
  time(&rawtime);
  timeinfo            = localtime(&rawtime); 
  int captureCur      = timeinfo->tm_sec;
  int captureDelt     = captureCur - captureLast;
  if (captureDelt < 0)
    captureDelt       = captureDelt + 60;
  if (captureDelt > captureRate) {
    fps               = (float)ticks/(float)captureDelt; 
    captureLast       = captureCur;
    ticks             = 0;
    printf("fps=%f\n", fps);
  }
  return fps;
}

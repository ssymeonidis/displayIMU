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

// function contains rotation matrix quaternion based on
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

// include statements
#include <stdio.h>
#include <math.h>
#include "quatForwardUp.h"

#define IS_ORTHO_NORMALIZE 1

void quatForwardUp(float* f, float* u, float* q)
{
  // defined internal variables
  float r[3];     // right vector
  float n;        // nomalized valued

  // normalize reference (up) vector
  #ifdef IS_ORTHO_NORMALIZE 
  n           = sqrt(u[0]*u[0]+u[1]*u[1]+u[2]*u[2]);
  u[0]       /= n;
  u[1]       /= n;
  u[2]       /= n;

  // ortho-normalize forwared vector 
  n           = f[0]*u[0]+f[1]*u[1]+f[2]*u[2];
  f[0]        = f[0]-n*u[0];
  f[1]        = f[1]-n*u[1];
  f[2]        = f[2]-n*u[2];
  n           = sqrt(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
  f[0]       /= n;
  f[1]       /= n;
  f[2]       /= n;
  #endif

  // calcuate the right vector
  r[0]        = u[1]*f[2]-u[2]*f[1];
  r[1]        = u[2]*f[0]-u[0]*f[2];
  r[2]        = u[0]*f[1]-u[1]*f[0]; 

  // calculate the quaternion
  n           = f[0]+r[1]+u[2];
  if (n > 0) {
    n         = sqrt(1.0+n)*2;
    q[0]      = 0.25*n;
    q[1]      = (u[1]-r[2])/n;
    q[2]      = (f[2]-u[0])/n;
    q[3]      = (r[0]-f[1])/n;
  } else if (f[0] > r[1] && f[0] > u[2]) {
    n         = sqrt(1.0+f[0]-r[1]-u[2])*2;
    q[0]      = (u[1]-r[2])/n;
    q[1]      = 0.25*n;
    q[2]      = (f[1]+r[0])/n;
    q[3]      = (f[2]+u[0])/n;
  } else if (r[1] > u[2]) {
    n         = sqrt(1.0+r[1]-f[0]-u[2])*2;
    q[0]      = (f[2]-u[0])/n;
    q[1]      = (f[1]+r[0])/n;
    q[2]      = 0.25*n;
    q[3]      = (r[2]+u[1])/n;
  } else {
    n         = sqrt(1.0+u[2]-f[0]-r[1])*2;
    q[0]      = (r[0]-f[1])/n;
    q[1]      = (f[2]+u[0])/n;
    q[2]      = (r[2]+u[1])/n;
    q[3]      = 0.25*n;
  }
}

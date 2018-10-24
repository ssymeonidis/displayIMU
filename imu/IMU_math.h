/*
 * This file is part of quaternion-based displayIMU C/C/C++/QT code base
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

#ifndef _IMU_MATH_H
#define _IMU_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

// basic quaternion operators
float* IMU_math_quatMult      (float *in1, float *in2, float *out);
float* IMU_math_quatMultConj  (float *in1, float *in2, float *out);
float* IMU_math_rotateForward (float *v,   float *q,   float *out);
float* IMU_math_rotateReverse (float *v,   float *q,   float *out);

// converting between quaternions and Euler angles
float* IMU_math_quatToEuler   (float *q,   float *E);
float* IMU_math_eulerToQuat   (float *E,   float *q);
float* IMU_math_radToDeg      (float *r,   float *d);
float* IMU_math_degToRad      (float *d,   float *r);

// converting between quaternions and pointing vectors
float* IMU_math_quatToUp      (float *q,   float *v);
float* IMU_math_quatToFrwd    (float *q,   float *v);
float* IMU_math_upToQuat      (float *u,   float *q);
float* IMU_math_upFrwdToQuat  (float *u,   float *f,   float *q);
float* IMU_math_vectToQuat    (float *u,   float *v,   float *q);

// core filters
int    IMU_math_estmAccl      (float *q, float *a, float alpha, float *FOM);
int    IMU_math_estmMagnRef   (float *q, float *m, float refx,  float refz,
                               float alpha, float *FOM);
float  IMU_math_calcWeight    (float val, float ref, float thresh);

#ifdef __cplusplus
}
#endif

#endif

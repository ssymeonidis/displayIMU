/*
 * This file is part of quaternion-based displayIMU C/C/C++/QT code base
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

#ifndef _IMU_MATH_H
#define _IMU_MATH_H

#ifdef __cplusplus
extern "C" {
#endif

// functions for modifying/converting quaternions
void IMU_math_calcEuler     (float* q, float* E);
void IMU_math_calcRefState  (float* q, float* ref);
void IMU_math_calcRefVectUp (float* u, float* ref);
void IMU_math_applyRef      (float* q, float* ref, float* q_out);

#ifdef __cplusplus
}
#endif

#endif

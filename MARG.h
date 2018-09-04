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

#ifndef _MARG_H
#define _MARG_H

// tuning params
extern float accl_scale;
extern float mag_scale;
extern float gyro_scale;
extern float accl_alpha;
extern float zeta;

// sensor biases
extern float aBiasX;
extern float aBiasY;
extern float aBiasZ;
extern float mBiasX;
extern float mBiasY;
extern float mBiasZ;
extern float gBiasX;
extern float gBiasY;
extern float gBiasZ;

// calibration magnitudes/angles
extern float G;
extern float M;
extern float ang;

// debug values
extern float delta_G;
extern float delta_a;
extern float delta_m;
extern float delta_M;
extern float delta_ang;

// disable controls
extern int isAcclDisable;
extern int isMagDisable;
extern int isGyroDisable;

void initMARG(int is_csv_file);
void refAcclMARG(float* a);
void updateMARG(float* g, float* a, float* m, float* E, float* A, 
                int isRef, int isReset, int isCalib);

#endif


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

/*
 * This function is based off the work performed by Sebastian O.H. Madgwick, 
 * documented in the paper "An efficient orientation Filter for inertial and
 * inertial/magnetic sensor arrays. Changes were made to turn on/off sensors 
 * and to adapted the filter for human activity recoginition.  Significant
 * changes were made for adding hooks and increasing readabilty 
*/

// Math library required for sqrt
#include <math.h>
#include <stdio.h>
#include "quatForwardUp.h"
#include "MARG.h"

// System constants
float accl_scale    = 0.00375;
float mag_scale     = 0.00375;
float gyro_scale    = 0;   
float accl_alpha    = 0.7;
float zeta          = 0.0015;

// sensor bias
float aBiasX        = 0.0;
float aBiasY        = 0.0;
float aBiasZ        = 0.0;
float mBiasX        = 0.0;
float mBiasY        = 0.0;
float mBiasZ        = 0.0;
float gBiasX        = 0.0;
float gBiasY        = 0.0;
float gBiasZ        = 0.0; 

// estimated orientation quaternion
float SEq_1         = 1.0;
float SEq_2         = 0.0;
float SEq_3         = 0.0;
float SEq_4         = 0.0;

// estimated acceleration vector
float A_x           = 0.0;
float A_y           = 0.0;
float A_z           = 0.0;

// refernce quaternion
float ref_1         = 1.0;
float ref_2         = 0.0;
float ref_3         = 0.0;
float ref_4         = 0.0;

// reference magnitude
float G             = 1;
float G_est         = G;
float M             = 1;
float ang           = -0.643;

// reference direction of flux 
float b_x           = 0.57;
float b_z           = 0.82; 

// estimate gyroscope biases error
float w_bx          = 0;
float w_by          = 0;
float w_bz          = 0;

// debug variables
float error_fltr    = 0.8;
float delta_G       = 0;
float delta_a       = 0;
float delta_m       = 0;
float delta_M       = 0;
float delta_ang     = 0;

// Control variables
int isAccl          = 1;
int isMag           = 1;
int isGyro          = 1;
int isGrad          = 1;
int isFltr          = 1;
int isBias          = 0;
int isFlux          = 0;
int isAcclDisable   = 0;
int isMagDisable    = 0;
int isGyroDisable   = 0;

int is_csv_file     = 0;

// Function to initialize filter
void initMARG(int is_csv_file_in)
{
  is_csv_file = is_csv_file_in;
  if (is_csv_file == 0) {
    gyro_scale      = 315;   
    G               = 10.48;
    M               = 50;
  } else {
    gyro_scale      = 960000; 
    G               = 16930.7;
    M               = 2700;
  }
  G_est             = G;
}

// Function to set accl reference
void refAcclMARG(float* a)
{
  // assining the variables
  float a_x, a_y, a_z;
  float tmp;
  if (is_csv_file == 0) {
    a_x             = a[0];
    a_y             = a[1];
    a_z             = a[2];
  } else {
    a_x             = -a[0];
    a_y             =  a[1];
    a_z             =  a[2];
  }

  if (a_x*a_x+a_y*a_y+a_z*a_z < 1.0) {
    printf("invalid accelerometer vector... setting to global coordinates\n");
    a_x             = 0.0;
    a_y             = 0.0;
    a_z  	    = 1.0;
  }
  tmp               = sqrt(a_x*a_x+a_y*a_y+a_z*a_z);
  a_x              /= tmp;
  a_y              /= tmp;
  a_z              /= tmp;
  tmp               =  sqrt(2.0 + 2.0 * a_z);
  if (tmp > 0.001) {
    ref_1           =  0.5 * tmp;
    ref_2           = -a_y / tmp;
    ref_3           =  a_x / tmp;
    ref_4           =  0.0;
  } else {
    ref_1           =  0.0;
    ref_2           =  0.0;
    ref_3           =  1.0;
    ref_4           =  0.0;
  }
}

// Function to compute one filter iteration
void updateMARG(float* g, float* a, float* m, float* E, float* A, 
                int isRef, int isReset, int isCalib)
{
  // assining the variables
  float w_x, w_y, w_z;
  float a_x, a_y, a_z;
  float m_x, m_y, m_z;
  if (is_csv_file == 0) {
    w_x                  = g[0] - gBiasX;
    w_y                  = g[1] - gBiasY;
    w_z                  = g[2] - gBiasZ;
    a_x                  = a[0] - aBiasX;
    a_y                  = a[1] - aBiasY;
    a_z                  = a[2] - aBiasZ;
    m_x                  = m[0] - mBiasX;
    m_y                  = m[1] - mBiasY;
    m_z                  = m[2] - mBiasZ;
  } else {
    w_x                  = -g[1] + gBiasY;
    w_y                  = -g[0] + gBiasX;
    w_z                  = -g[2] + gBiasZ;
    a_x                  = -a[0] + aBiasX;
    a_y                  =  a[1] - aBiasY;
    a_z                  =  a[2] - aBiasZ;
    m_x                  = -m[0] + mBiasX;
    m_y                  =  m[1] - mBiasY;
    m_z                  =  m[2] - mBiasZ;
  }

  // magnitometer up vector
  float up_x             = 0.0;
  float up_y             = 0.0;
  float up_z             = 0.0;

  // down reference vector
  float G_x              = 0.0;
  float G_y              = 0.0;
  float G_z              = 0.0;
  float G_Q1             = 0.0;
  float G_Q2             = 0.0;
  float G_Q3             = 0.0;
  float G_Q4             = 0.0;
  
  // acceleration results
  float A_1              = 0.0;
  float A_2              = 0.0;
  float A_3              = 0.0;

  // relative quaterion result
  float SSq_1            = 0.0;
  float SSq_2            = 0.0;
  float SSq_3            = 0.0;
  float SSq_4            = 0.0;

  // objective function elements
  float f_1              = 0.0;
  float f_2              = 0.0;
  float f_3              = 0.0;
  float f_4              = 0.0;
  float f_5              = 0.0;
  float f_6              = 0.0;

  // objective function Jacobian elements
  float J_11or24         = 0.0;
  float J_12or23         = 0.0;
  float J_13or22         = 0.0;
  float J_14or21         = 0.0;
  float J_32             = 0.0;
  float J_33             = 0.0;
  float J_41             = 0.0;
  float J_42             = 0.0;
  float J_43             = 0.0;
  float J_44             = 0.0;
  float J_51             = 0.0;
  float J_52             = 0.0;
  float J_53             = 0.0;
  float J_54             = 0.0;
  float J_61             = 0.0;
  float J_62             = 0.0;
  float J_63             = 0.0;
  float J_64             = 0.0;

  // estimated direction of the gyroscope error
  float SEqHatDot_1      = 0.0;
  float SEqHatDot_2      = 0.0; 
  float SEqHatDot_3      = 0.0; 
  float SEqHatDot_4      = 0.0;

  // estimated direction of the gyroscope error (angular)
  float w_err_x          = 0.0;
  float w_err_y          = 0.0;
  float w_err_z          = 0.0;
  
  // computed flux in the earth frame
  float h_x              = 0.0; 
  float h_y              = 0.0; 
  float h_z              = 0.0; 
  
  // axulirary variables to avoid reapeated calcualtions
  float halfSEq_1        = 0.5f * SEq_1;
  float halfSEq_2        = 0.5f * SEq_2;
  float halfSEq_3        = 0.5f * SEq_3;
  float halfSEq_4        = 0.5f * SEq_4;
  float twoSEq_1         = 2.0f * SEq_1;
  float twoSEq_2         = 2.0f * SEq_2;
  float twoSEq_3         = 2.0f * SEq_3;
  float twoSEq_4         = 2.0f * SEq_4;
  float twob_x           = 2.0f * b_x;
  float twob_z           = 2.0f * b_z;
  float twob_xSEq_1      = 2.0f * b_x * SEq_1;
  float twob_xSEq_2      = 2.0f * b_x * SEq_2;
  float twob_xSEq_3      = 2.0f * b_x * SEq_3;
  float twob_xSEq_4      = 2.0f * b_x * SEq_4;
  float twob_zSEq_1      = 2.0f * b_z * SEq_1;
  float twob_zSEq_2      = 2.0f * b_z * SEq_2;
  float twob_zSEq_3      = 2.0f * b_z * SEq_3;
  float twob_zSEq_4      = 2.0f * b_z * SEq_4;
  float SEq_1SEq_2       = 0.0;
  float SEq_1SEq_3       = SEq_1 * SEq_3;
  float SEq_1SEq_4       = 0.0;
  float SEq_2SEq_3       = 0.0;
  float SEq_2SEq_4       = SEq_2 * SEq_4;
  float SEq_3SEq_4       = 0.0;
  float twom_x           = 0.0;
  float twom_y           = 0.0;
  float twom_z           = 0.0;
  float SQq_1            = 0.0;
  float SQq_2            = 0.0; 
  float SQq_3            = 0.0;
  float SQq_4            = 0.0;
  float D                = 0.0;
  float norm             = 0.0;
  float tmp              = 0.0; 


  if (isGyro != 0) { 

    /**************************************************************************
    * apply gyroscope rates
    **************************************************************************/

    // normalize the gyroscope measurement
    norm           = gyro_scale/(2*M_PI);
    w_x           /= norm;
    w_y           /= norm;
    w_z           /= norm;

    // remove the gyroscope baises
    w_x           -= w_bx;
    w_y           -= w_by;
    w_z           -= w_bz;

    // compute gyro quaternion rate and apply delta to estimated orientation 
    SEq_1         += -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
    SEq_2         +=  halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
    SEq_3         +=  halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
    SEq_4         +=  halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

    /**************************************************************************
    * calcuate acceleration vector
    **************************************************************************/
  
    // rotate gravity up vector by estimated orientation
    G_Q1           = -SEq_4 * G;
    G_Q2           = -SEq_3 * G;
    G_Q3           =  SEq_2 * G;
    G_Q4           =  SEq_1 * G;
    G_x            = -G_Q1 * SEq_2 + G_Q2 * SEq_1 + G_Q3 * SEq_4 - G_Q4 * SEq_3;
    G_y            = -G_Q1 * SEq_3 - G_Q2 * SEq_4 + G_Q3 * SEq_1 + G_Q4 * SEq_2;
    G_z            = -G_Q1 * SEq_4 + G_Q2 * SEq_3 - G_Q3 * SEq_2 + G_Q4 * SEq_1;
 
    // remove the gravity from the accelerometer data
    if (isReset != 0) {
      A_1          = a_x-G_x;
      A_2          = a_y-G_y;
      A_3          = a_z-G_z;
    } else {
      A_1          = accl_alpha*A_x + (1.0-accl_alpha)*(a_x-G_x);
      A_2          = accl_alpha*A_y + (1.0-accl_alpha)*(a_y-G_y);
      A_3          = accl_alpha*A_z + (1.0-accl_alpha)*(a_z-G_z); 
    }

    // estimate the gravity based on weighted average of all samples
    norm           = sqrt(a_x*a_x + a_y*a_y + a_z*a_z);
    G_est          = zeta * G_est + (1.0-zeta) * norm; 
    norm           = 100*(norm-G)/G;
    delta_G        = error_fltr*delta_G + (1-error_fltr)*norm; 

    /**************************************************************************
    * calcuate magnetometer quality metrics
    **************************************************************************/
  
    // calcuate the gravity magnitude error
    norm           = sqrt(m_x*m_x + m_y*m_y + m_z*m_z);
    norm           = 100*(norm-M)/M;
    delta_M        = error_fltr*delta_M + (1-error_fltr)*norm; 

    // calcuate the gravity angle (minux the arccos function)
    norm           =  (a_x*m_x + a_y*m_y + a_z*m_z) /
                     (sqrt(a_x*a_x + a_y*a_y + a_z*a_z) *
                      sqrt(m_x*m_x + m_y*m_y + m_z*m_z));
    norm           = 100*(norm-ang)/ang;
    delta_ang      = error_fltr*delta_ang + (1-error_fltr)*norm; 
  }

  /****************************************************************************
  * normalize inputs
  ****************************************************************************/

  // normalize the accelerometer measurement
  norm       = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
  a_x       /= norm;
  a_y       /= norm;
  a_z       /= norm;

  // normalize the magnetometer measurement
  if (isFlux == 0) {
    D        = m_x * a_x + m_y * a_y + m_z * a_z; 
    up_x     = D * a_x;
    up_y     = D * a_y;
    up_z     = D * a_z;
    m_x     -= up_x;
    m_y     -= up_y;
    m_z     -= up_z;
  }
  norm       = sqrt(m_x * m_x + m_y * m_y + m_z * m_z); 
  m_x       /= norm; 
  m_y       /= norm; 
  m_z       /= norm;
  twom_x     = 2.0 * m_x;
  twom_y     = 2.0 * m_y;
  twom_z     = 2.0 * m_z;

  /****************************************************************************
  * dead recokon orientation w/ accelerometer & accelerometer
  ****************************************************************************/

  if ((isGrad == 0 && isAccl != 0 && isMag != 0) || 
      (isReset != 0) || (isGyroDisable != 0)) {
    float q[4];
    float f[4]  = {m_x, m_y, m_z};
    float u[4]  = {a_x, a_y, a_z};
    quatForwardUp(f, u, q);
    SEq_1       = q[0];
    SEq_2       = q[1];
    SEq_3       = q[2];
    SEq_4       = q[3];
  }


  /****************************************************************************
  * dead recokon orientation w/ accelerometer
  ****************************************************************************/

  if (isAccl != 0 && isReset == 0 && isGyroDisable == 0) {

    // calculate the shortest rotation quaternion
    if (isGrad == 0 && isMag == 0) {
      tmp       =  sqrt(2.0 + 2.0 * a_z);
      SEq_1     =  0.5 * tmp;
      SEq_2     =  a_y / tmp;
      SEq_3     = -a_x / tmp;
      SEq_4     =  0.0;

    } else if (isGrad != 0) {

      // compute the objective function 
      f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
      f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
      f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z; 
 
      // compute the Jacobian
      J_11or24 = twoSEq_3; 
      J_12or23 = 2.0f * SEq_4;
      J_13or22 = twoSEq_1;
      J_14or21 = twoSEq_2;
      J_32     = 2.0f * J_14or21;
      J_33     = 2.0f * J_11or24;

      // calculate the gradient
      SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
      SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32     * f_3;
      SEqHatDot_3 = J_12or23 * f_2 - J_33     * f_3 - J_13or22 * f_1;
      SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;

      // normalise the gradient to estimate direction of the gyroscope error
      norm        = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 +
                         SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
      SEqHatDot_1 = SEqHatDot_1 / norm;
      SEqHatDot_2 = SEqHatDot_2 / norm;
      SEqHatDot_3 = SEqHatDot_3 / norm;
      SEqHatDot_4 = SEqHatDot_4 / norm;

      // integrate the estimated quaternion rate
      SEq_1 -= accl_scale * SEqHatDot_1;
      SEq_2 -= accl_scale * SEqHatDot_2;
      SEq_3 -= accl_scale * SEqHatDot_3;
      SEq_4 -= accl_scale * SEqHatDot_4;

      // calculate delta metric
      delta_a = SEqHatDot_1;
    }
     
  }

  /****************************************************************************
  * dead recokon orientation w/ magnetometer
  ****************************************************************************/

  if (isMag != 0 && isMagDisable == 0 && isReset == 0 && isGyroDisable == 0) {

    if (isGrad == 0 && isAccl == 0) {
      tmp         =  sqrt(2.0 + 2.0 * m_x);
      SEq_1       =  0.5 * tmp;
      SEq_2       =  0.0;
      SEq_3       =  m_z / tmp;
      SEq_4       = -m_y / tmp;

    } else if (isGrad != 0) { 

      // compute the objective function 
      f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + 
            twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
      f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + 
            twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
      f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + 
            twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;

      // compute the Jacobian
      J_41 = twob_zSEq_3;
      J_42 = twob_zSEq_4;
      J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; 
      J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;
      J_51 = twob_xSEq_4 - twob_zSEq_2;
      J_52 = twob_xSEq_3 + twob_zSEq_1;
      J_53 = twob_xSEq_2 + twob_zSEq_4;
      J_54 = twob_xSEq_1 - twob_zSEq_3;
      J_61 = twob_xSEq_3;
      J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
      J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
      J_64 = twob_xSEq_2;

      // calculate the gradient
      SEqHatDot_1 = - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
      SEqHatDot_2 =   J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
      SEqHatDot_3 = - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
      SEqHatDot_4 = - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;

      // normalise the gradient to estimate direction of the gyroscope error
      norm        = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 +
                         SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
      SEqHatDot_1 = SEqHatDot_1 / norm;
      SEqHatDot_2 = SEqHatDot_2 / norm;
      SEqHatDot_3 = SEqHatDot_3 / norm;
      SEqHatDot_4 = SEqHatDot_4 / norm;

      // integrate the estimated quaternion rate
      SEq_1 -= mag_scale * SEqHatDot_1;
      SEq_2 -= mag_scale * SEqHatDot_2;
      SEq_3 -= mag_scale * SEqHatDot_3;
      SEq_4 -= mag_scale * SEqHatDot_4;

      // calculate delta metric
      delta_m = SEqHatDot_1;
    }
  }

  /****************************************************************************
  * convert orientation to coordinate system
  ****************************************************************************/

  // normalise quaternion
  norm   = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + 
                SEq_3 * SEq_3 + SEq_4 * SEq_4);
  SEq_1 /= norm;
  SEq_2 /= norm;
  SEq_3 /= norm;
  SEq_4 /= norm;

  // capture reference if selected
  if (isRef != 0) {
    ref_1 =  SEq_1;
    ref_2 = -SEq_2;
    ref_3 = -SEq_3;
    ref_4 = -SEq_4;
  }
  
  // apply reference
  SSq_1   = SEq_1*ref_1 - SEq_2*ref_2 - SEq_3*ref_3 - SEq_4*ref_4;
  SSq_2   = SEq_1*ref_2 + SEq_2*ref_1 + SEq_3*ref_4 - SEq_4*ref_3;
  SSq_3   = SEq_1*ref_3 - SEq_2*ref_4 + SEq_3*ref_1 + SEq_4*ref_2;
  SSq_4   = SEq_1*ref_4 + SEq_2*ref_3 - SEq_3*ref_2 + SEq_4*ref_1;

  // apply rotation to acceleration vector
  if (isAcclDisable == 0 && isGyro != 0) {
    float tmp_1 = -SEq_2*A_1 - SEq_3*A_2 - SEq_4*A_3;
    float tmp_2 =  SEq_1*A_1 + SEq_3*A_3 - SEq_4*A_2;
    float tmp_3 =  SEq_1*A_2 - SEq_2*A_3 + SEq_4*A_1;
    float tmp_4 =  SEq_1*A_3 + SEq_2*A_2 - SEq_3*A_1;
    A[0]    = -tmp_1*SEq_2 + tmp_2*SEq_1 - tmp_3*SEq_4 + tmp_4*SEq_3;
    A[1]    = -tmp_1*SEq_3 + tmp_2*SEq_4 + tmp_3*SEq_1 - tmp_4*SEq_2;
    A[2]    = -tmp_1*SEq_4 - tmp_2*SEq_3 + tmp_3*SEq_2 + tmp_4*SEq_1;
  } else {
    A[0]    = 0.0;
    A[1]    = 0.0;
    A[2]    = 0.0;
  }
 
  // calcuate reference angles
  SQq_1 = SSq_1 * SSq_1;
  SQq_2 = SSq_2 * SSq_2;
  SQq_3 = SSq_3 * SSq_3;
  SQq_4 = SSq_4 * SSq_4;
  E[0]  = 180 * atan2(2*(SSq_2*SSq_3+SSq_4*SSq_1),  SQq_2-SQq_3-SQq_4+SQq_1) / M_PI;
  E[1]  = 180 * asin(-2*(SSq_2*SSq_4-SSq_3*SSq_1)) / M_PI;
  E[2]  = 180 * atan2(2*(SSq_3*SSq_4+SSq_2*SSq_1), -SQq_2-SQq_3+SQq_4+SQq_1) / M_PI;

  /****************************************************************************
  * update feedback parameters (gyro bias and flux vector)
  ****************************************************************************/

  if (isCalib != 0) {
    w_bx    = w_x;
    w_by    = w_y;
    w_bz    = w_z;
  } 

  // compute angular estimated direction of the gyroscope error
  w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - 
            twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
  w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - 
            twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
  w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + 
            twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;

  // filter errors to estimate bias 
  if (isGyro != 0 && isBias != 0) {
    w_bx   += w_err_x * zeta;
    w_by   += w_err_y * zeta;
    w_bz   += w_err_z * zeta;
  }

  if (isMag  != 0 && isFlux != 0) {
    // compute flux in the earth frame
    SEq_1SEq_2 = SEq_1 * SEq_2; 
    SEq_1SEq_3 = SEq_1 * SEq_3;
    SEq_1SEq_4 = SEq_1 * SEq_4;
    SEq_3SEq_4 = SEq_3 * SEq_4;
    SEq_2SEq_3 = SEq_2 * SEq_3;
    SEq_2SEq_4 = SEq_2 * SEq_4;
    h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + 
          twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + 
          twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + 
          twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + 
          twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + 
          twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + 
          twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);

    // normalise the flux vector to have only components in the x and z
    b_x = sqrt((h_x * h_x) + (h_y * h_y));
    b_z = h_z; 
  } else {
    b_x = 1.0;
    b_z = 0.0;
  } 

}

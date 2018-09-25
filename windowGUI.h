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

#ifndef WINDOW_GUI_H
#define WINDOW_GUI_H

#include <QWidget>
#include "glWidget.h"
#include "IMU.h"

// QT class defintions
class QTimer;
class QSlider;
class GLWidget;
class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;

// main class definition
class WindowGUI : public QWidget
{
  Q_OBJECT

  public:
                  WindowGUI(const char* config_file, const char* calib_file);
    void          setRefAccl(float* a);
    void          setParams(float* params);

  public slots:
    void          updateDebug();
    void          updateCheckAccl();
    void          updateCheckMag();
    void          updateCheckGyro();
    void          updateCheckIMU();
    void          updateStateIMU();
    void          updateViewUp();
    void          updateViewSide1();
    void          updateViewSide2();
    void          updateReference();
    void          updateResetIMU();
    void          updateCalibIMU();
    void          updateRefAccl();

  protected:
    void          keyPressEvent(QKeyEvent *event);

  private:
    // private function 
    QSlider*      createSlider();

    // internal objects/variables
    GLWidget*           glWidget;
    QTimer*             refreshTimer;
    displayIMU_calib*   calib;
    displayIMU_config*  config;
    int                 is_csv_file;

    // GUI objects 
    QSlider*      xSlider;
    QSlider*      ySlider;
    QSlider*      zSlider;
    QCheckBox*    acclCheckBox;
    QLabel*       acclText;
    QLineEdit*    acclScale;  
    QCheckBox*    magCheckBox;
    QLabel*       magText;
    QLineEdit*    magScale;  
    QCheckBox*    gyroCheckBox;
    QLabel*       gyroText;
    QLineEdit*    gyroScale;  
    QCheckBox*    isIMU;
    QLabel*       isIMUText;
    QLineEdit*    isIMUScale;
    QCheckBox*    isAccl;
    QLabel*       isAcclText;
    QCheckBox*    isMag;
    QLabel*       isMagText;
    QCheckBox*    isGyro;
    QLabel*       isGyroText;
    QLabel*       refAccl;
    QLineEdit*    refAcclX;
    QLineEdit*    refAcclY;
    QLineEdit*    refAcclZ;
    QPushButton*  upButton;
    QPushButton*  sideButton1;
    QPushButton*  sideButton2; 
    QPushButton*  refButton;
    QPushButton*  resetButton;
    QPushButton*  calibButton;
    QLabel*       acclRateText;
    QLineEdit*    acclRate;
    QLabel*       magRateText;
    QLineEdit*    magRate;
    QLabel*       acclAlphaText;
    QLineEdit*    acclAlpha;
    QLabel*       zetaText;
    QLineEdit*    zetaValue;
    QLabel*       yawText;
    QLabel*       yaw;
    QLabel*       pitchText;
    QLabel*       pitch;
    QLabel*       rollText;
    QLabel*       roll;
    QLabel*       acclXText;
    QLabel*       acclX;
    QLabel*       acclYText;
    QLabel*       acclY;
    QLabel*       acclZText;
    QLabel*       acclZ;  
    QLabel*       deltaGravText;
    QLabel*       deltaGrav;
    QLabel*       deltaNormText;
    QLabel*       deltaNorm;
    QLabel*       deltaAngText;
    QLabel*       deltaAng;
    QLabel*       deltaAcclText;
    QLabel*       deltaAccl;
    QLabel*       deltaMagText;
    QLabel*       deltaMag;
    QLabel*       biasAcclXText;
    QLineEdit*    biasAcclX;
    QLabel*       biasAcclYText;
    QLineEdit*    biasAcclY;
    QLabel*       biasAcclZText;
    QLineEdit*    biasAcclZ;
    QLabel*       acclMagText;
    QLineEdit*    acclMag;
    QLabel*       biasMagXText;
    QLineEdit*    biasMagX;
    QLabel*       biasMagYText;
    QLineEdit*    biasMagY;
    QLabel*       biasMagZText;
    QLineEdit*    biasMagZ;
    QLabel*       magMagText;
    QLineEdit*    magMag;
    QLabel*       magAngText;
    QLineEdit*    magAng;
    QLabel*       biasGyroXText;
    QLineEdit*    biasGyroX;
    QLabel*       biasGyroYText;
    QLineEdit*    biasGyroY;
    QLabel*       biasGyroZText;
    QLineEdit*    biasGyroZ;
    QLabel*       gyroRateText;
    QLineEdit*    gyroRate;
};

#endif

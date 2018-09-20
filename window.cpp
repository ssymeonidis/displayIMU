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

#include <QtGui>
#include <QtWidgets>
#include "window.h"
#include "receive.h"
#include "dataread.h"

Window::Window(int is_csv_file_in, const char* video_path, int offset, float rate, int max_frame)
{
  // save system state
  is_csv_file       = is_csv_file_in;

  // create display/video widgets
  glWidget          = new GLWidget(is_csv_file);
  if (video_path != NULL)
    video           = new pixmap(video_path, offset, rate, max_frame);
  displayIMU_getCalib(&calib);
  displayIMU_getConfig(&config);

  // create timer
  refreshTimer = new QTimer(this);
  connect(refreshTimer, SIGNAL(timeout()), this, SLOT(updateDebug()));
  refreshTimer->start(150);

  // create control widgets
  xSlider           = createSlider();
  ySlider           = createSlider();
  zSlider           = createSlider();
  acclCheckBox      = new QCheckBox();
  acclCheckBox->setChecked(glWidget->isAccl);
  acclText          = new QLabel();
  acclText->setText("acclerometer");
  acclScale         = new QLineEdit(QString::number(glWidget->scaleAccl));
  magCheckBox       = new QCheckBox();
  magCheckBox->setChecked(glWidget->isMag);
  magText           = new QLabel();
  magText->setText("magnetometer");
  magScale          = new QLineEdit(QString::number(glWidget->scaleMag));
  gyroCheckBox      = new QCheckBox();
  gyroCheckBox->setChecked(glWidget->isGyro);
  gyroText          = new QLabel();
  gyroText->setText("gyroscope");
  gyroScale         = new QLineEdit(QString::number(glWidget->scaleGyro));
  isIMU             = new QCheckBox();
  isIMUText         = new QLabel();
  isIMUText->setText("IMU");
  isIMUScale        = new QLineEdit(QString::number(glWidget->scaleIMU));
  isAccl            = new QCheckBox();
  isAccl->setFixedSize(QSize(29, 29));
  isAcclText        = new QLabel();
  isAcclText->setText("No Accl");
  isAcclText->setFixedSize(QSize(72, 29));
  isMag             = new QCheckBox();
  isMag->setFixedSize(QSize(29, 29));
  isMagText         = new QLabel();
  isMagText->setText("No Mag"); 
  isMagText->setFixedSize(QSize(72, 29));
  isGyro            = new QCheckBox();
  isGyro->setFixedSize(QSize(29, 29));
  isGyroText        = new QLabel();
  isGyroText->setText("No Gyro");
  isGyroText->setFixedSize(QSize(72, 29));
  refAccl           = new QLabel();
  refAccl->setText("  up");
  refAcclX          = new QLineEdit(QString::number(0));
  refAcclX->setMaximumWidth(75);
  refAcclY          = new QLineEdit(QString::number(0));
  refAcclY->setMaximumWidth(75);
  refAcclZ          = new QLineEdit(QString::number(1));
  refAcclZ->setMaximumWidth(75);
  upButton          = new QPushButton("up");
  upButton->setFixedSize(QSize(60, 29));
  sideButton1       = new QPushButton("side1");
  sideButton1->setFixedSize(QSize(60, 29));
  sideButton2       = new QPushButton("side2");
  sideButton2->setFixedSize(QSize(60, 29));
  resetButton       = new QPushButton("reset");
  resetButton->setFixedSize(QSize(60, 29));
  refButton         = new QPushButton("set reference");
  refButton->setFixedSize(QSize(110, 29));
  calibButton       = new QPushButton("calib");
  calibButton->setFixedSize(QSize(60, 29));
  acclRateText      = new QLabel();
  acclRateText->setText("Accl Beta");
  acclRate          = new QLineEdit(QString::number(config->aWeight));
  magRateText       = new QLabel();
  magRateText->setText("Mag Beta");
  magRate           = new QLineEdit(QString::number(config->mWeight));
  acclAlphaText     = new QLabel();
  acclAlphaText->setText("Accl Alpha");
  acclAlpha         = new QLineEdit(QString::number(config->acclAlpha)); 
  zetaText          = new QLabel();
  zetaText->setText("Zeta");
  zetaValue         = new QLineEdit(QString::number(config->autocalAlpha2));
  yawText           = new QLabel();
  yawText->setText("yaw:");
  yaw               = new QLabel();
  pitchText         = new QLabel();
  pitchText->setText("pitch:");
  pitch             = new QLabel();
  rollText          = new QLabel();
  rollText->setText("roll:");
  roll              = new QLabel();
  acclXText         = new QLabel();
  acclXText->setText("accl X:");
  acclX             = new QLabel();
  acclYText         = new QLabel();
  acclYText->setText("accl Y:");
  acclY             = new QLabel();
  acclZText         = new QLabel();
  acclZText->setText("accl Z:");
  acclZ             = new QLabel();
  deltaGravText     = new QLabel();
  deltaGravText->setText("Delta Gravity(%):");
  deltaGrav         = new QLabel();
  deltaNormText     = new QLabel();
  deltaNormText->setText("Delta Norm(%):");
  deltaNorm         = new QLabel();
  deltaAngText      = new QLabel();
  deltaAngText->setText("Delta Ang(%):"); 
  deltaAng          = new QLabel();
  deltaAcclText     = new QLabel();
  deltaAcclText->setText("Delta Accl:");
  deltaAccl         = new QLabel();
  deltaMagText      = new QLabel();
  deltaMagText->setText("Delta MaG:");
  deltaMag          = new QLabel();
  biasAcclXText     = new QLabel();
  biasAcclXText     = new QLabel();
  biasAcclXText->setText("Accl Bias X");
  biasAcclX         = new QLineEdit(QString::number(calib->aBias[0]));
  biasAcclYText     = new QLabel();
  biasAcclYText->setText("Accl Bias Y");
  biasAcclY         = new QLineEdit(QString::number(calib->aBias[1]));
  biasAcclZText     = new QLabel();
  biasAcclZText->setText("Accl Bias Z");
  biasAcclZ         = new QLineEdit(QString::number(calib->aBias[2]));
  acclMagText       = new QLabel();
  acclMagText->setText("Accl Mag");
  acclMag           = new QLineEdit(QString::number(calib->aMag));
  biasMagXText      = new QLabel();
  biasMagXText->setText("Mag Bias X");
  biasMagX          = new QLineEdit(QString::number(calib->mBias[0]));
  biasMagYText      = new QLabel();
  biasMagYText->setText("Mag Bias Y");
  biasMagY          = new QLineEdit(QString::number(calib->mBias[1]));
  biasMagZText      = new QLabel();
  biasMagZText->setText("Mag Bias Z");
  biasMagZ          = new QLineEdit(QString::number(calib->mBias[2]));
  magMagText        = new QLabel();
  magMagText->setText("Mag Mag");
  magMag            = new QLineEdit(QString::number(calib->mMag));
  magAngText        = new QLabel();
  magAngText->setText("Mag Angle");
  magAng            = new QLineEdit(QString::number(calib->mAng));
  biasGyroXText     = new QLabel();
  biasGyroXText->setText("Gyro Bias X");
  biasGyroX         = new QLineEdit(QString::number(calib->gBias[0]));
  biasGyroYText     = new QLabel();
  biasGyroYText->setText("Gyro Bias Y");
  biasGyroY         = new QLineEdit(QString::number(calib->gBias[1]));
  biasGyroZText     = new QLabel();
  biasGyroZText->setText("Gyro Bias Z");
  biasGyroZ         = new QLineEdit(QString::number(calib->gBias[2]));
  gyroRateText      = new QLabel();
  gyroRateText->setText("Gyro Rate");
  gyroRate          = new QLineEdit(QString::number(calib->gScale[0]));

  connect(xSlider,      SIGNAL(valueChanged(int)),     glWidget, SLOT(setXRotation(int)));
  connect(glWidget,     SIGNAL(xRotationChanged(int)), xSlider,  SLOT(setValue(int)));
  connect(ySlider,      SIGNAL(valueChanged(int)),     glWidget, SLOT(setYRotation(int)));
  connect(glWidget,     SIGNAL(yRotationChanged(int)), ySlider,  SLOT(setValue(int)));
  connect(zSlider,      SIGNAL(valueChanged(int)),     glWidget, SLOT(setZRotation(int)));
  connect(glWidget,     SIGNAL(zRotationChanged(int)), zSlider,  SLOT(setValue(int)));
  connect(acclCheckBox, SIGNAL(clicked()),             this,     SLOT(updateCheckAccl())); 
  connect(acclScale,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(magCheckBox,  SIGNAL(clicked()),             this,     SLOT(updateCheckMag())); 
  connect(magScale,     SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(gyroCheckBox, SIGNAL(clicked()),             this,     SLOT(updateCheckGyro())); 
  connect(gyroScale,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(isIMU,        SIGNAL(clicked()),             this,     SLOT(updateCheckIMU())); 
  connect(isIMUScale,   SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(isAccl,       SIGNAL(clicked()),             this,     SLOT(updateStateIMU()));
  connect(isMag,        SIGNAL(clicked()),             this,     SLOT(updateStateIMU()));
  connect(isGyro,       SIGNAL(clicked()),             this,     SLOT(updateStateIMU()));
  connect(refAcclX,     SIGNAL(editingFinished()),     this,     SLOT(updateRefAccl()));
  connect(refAcclY,     SIGNAL(editingFinished()),     this,     SLOT(updateRefAccl()));
  connect(refAcclZ,     SIGNAL(editingFinished()),     this,     SLOT(updateRefAccl()));
  connect(upButton,     SIGNAL(released()),            this,     SLOT(updateViewUp()));
  connect(sideButton1,  SIGNAL(released()),            this,     SLOT(updateViewSide1()));
  connect(sideButton2,  SIGNAL(released()),            this,     SLOT(updateViewSide2()));
  connect(refButton,    SIGNAL(released()),            this,     SLOT(updateReference()));
  connect(resetButton,  SIGNAL(released()),            this,     SLOT(updateResetIMU()));
  connect(calibButton,  SIGNAL(released()),            this,     SLOT(updateCalibIMU()));
  connect(acclRate,     SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(magRate,      SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(acclAlpha,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(zetaValue,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(biasAcclX,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(biasAcclY,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(biasAcclZ,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(acclMag,      SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(biasMagX,     SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(biasMagY,     SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(biasMagZ,     SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(magMag,       SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(magAng,       SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(biasGyroX,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(biasGyroY,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(biasGyroZ,    SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));
  connect(gyroRate,     SIGNAL(editingFinished()),     this,     SLOT(updateStateIMU()));

  QHBoxLayout *acclLayout = new QHBoxLayout;
  acclLayout->addWidget(acclCheckBox);
  acclLayout->addWidget(acclText);
  acclLayout->addWidget(acclScale);
  QHBoxLayout *magLayout = new QHBoxLayout;
  magLayout->addWidget(magCheckBox);
  magLayout->addWidget(magText);
  magLayout->addWidget(magScale);
  QHBoxLayout *gyroLayout = new QHBoxLayout;
  gyroLayout->addWidget(gyroCheckBox);
  gyroLayout->addWidget(gyroText);
  gyroLayout->addWidget(gyroScale);
  QHBoxLayout *imuLayout = new QHBoxLayout;
  imuLayout->addWidget(isIMU);
  imuLayout->addWidget(isIMUText);
  imuLayout->addWidget(isIMUScale);
  QHBoxLayout *disableLayout = new QHBoxLayout;
  disableLayout->addWidget(isAccl);
  disableLayout->addWidget(isAcclText);
  disableLayout->addWidget(isMag);
  disableLayout->addWidget(isMagText);
  disableLayout->addWidget(isGyro);
  disableLayout->addWidget(isGyroText);
  QHBoxLayout *refAcclLayout = new QHBoxLayout;
  refAcclLayout->addWidget(refAccl);
  refAcclLayout->addWidget(refAcclX);
  refAcclLayout->addWidget(refAcclY);
  refAcclLayout->addWidget(refAcclZ);
  QHBoxLayout *buttonLayout = new QHBoxLayout;
  buttonLayout->addWidget(upButton);
  buttonLayout->addWidget(sideButton1);
  buttonLayout->addWidget(sideButton2);
  QHBoxLayout *calibLayout = new QHBoxLayout;
  calibLayout->addWidget(refButton);
  calibLayout->addWidget(resetButton);
  calibLayout->addWidget(calibButton);
  QHBoxLayout *acclRateLayout = new QHBoxLayout;
  acclRateLayout->addWidget(acclRateText);
  acclRateLayout->addWidget(acclRate);
  QHBoxLayout *magRateLayout = new QHBoxLayout;
  magRateLayout->addWidget(magRateText);
  magRateLayout->addWidget(magRate);
  QHBoxLayout *acclAlphaLayout = new QHBoxLayout;
  acclAlphaLayout->addWidget(acclAlphaText);
  acclAlphaLayout->addWidget(acclAlpha);
  QHBoxLayout *zetaLayout = new QHBoxLayout;
  zetaLayout->addWidget(zetaText);
  zetaLayout->addWidget(zetaValue);
  QHBoxLayout *yawLayout = new QHBoxLayout;
  yawLayout->addWidget(yawText);
  yawLayout->addWidget(yaw);
  QHBoxLayout *pitchLayout = new QHBoxLayout;
  pitchLayout->addWidget(pitchText);
  pitchLayout->addWidget(pitch);
  QHBoxLayout *rollLayout = new QHBoxLayout;
  rollLayout->addWidget(rollText);
  rollLayout->addWidget(roll);
  QHBoxLayout *acclXLayout = new QHBoxLayout;
  acclXLayout->addWidget(acclXText);
  acclXLayout->addWidget(acclX);
  QHBoxLayout *acclYLayout = new QHBoxLayout;
  acclYLayout->addWidget(acclYText);
  acclYLayout->addWidget(acclY);
  QHBoxLayout *acclZLayout = new QHBoxLayout;
  acclZLayout->addWidget(acclZText);
  acclZLayout->addWidget(acclZ);
  QHBoxLayout *deltaGravLayout = new QHBoxLayout;
  deltaGravLayout->addWidget(deltaGravText);
  deltaGravLayout->addWidget(deltaGrav);
  QHBoxLayout *deltaNormLayout = new QHBoxLayout;
  deltaNormLayout->addWidget(deltaNormText);
  deltaNormLayout->addWidget(deltaNorm);
  QHBoxLayout *deltaAngLayout = new QHBoxLayout;
  deltaAngLayout->addWidget(deltaAngText);
  deltaAngLayout->addWidget(deltaAng);
  QHBoxLayout *deltaAcclLayout = new QHBoxLayout;
  deltaAcclLayout->addWidget(deltaAcclText);
  deltaAcclLayout->addWidget(deltaAccl);
  QHBoxLayout *deltaMagLayout = new QHBoxLayout;
  deltaMagLayout->addWidget(deltaMagText);
  deltaMagLayout->addWidget(deltaMag);
  QHBoxLayout *biasAcclXLayout = new QHBoxLayout;
  biasAcclXLayout->addWidget(biasAcclXText);
  biasAcclXLayout->addWidget(biasAcclX);
  QHBoxLayout *biasAcclYLayout = new QHBoxLayout;
  biasAcclYLayout->addWidget(biasAcclYText);
  biasAcclYLayout->addWidget(biasAcclY);
  QHBoxLayout *biasAcclZLayout = new QHBoxLayout;
  biasAcclZLayout->addWidget(biasAcclZText);
  biasAcclZLayout->addWidget(biasAcclZ);
  QHBoxLayout *acclMagLayout = new QHBoxLayout;
  acclMagLayout->addWidget(acclMagText);
  acclMagLayout->addWidget(acclMag);
  QHBoxLayout *biasMagXLayout = new QHBoxLayout;
  biasMagXLayout->addWidget(biasMagXText);
  biasMagXLayout->addWidget(biasMagX);
  QHBoxLayout *biasMagYLayout = new QHBoxLayout;
  biasMagYLayout->addWidget(biasMagYText);
  biasMagYLayout->addWidget(biasMagY);
  QHBoxLayout *biasMagZLayout = new QHBoxLayout;
  biasMagZLayout->addWidget(biasMagZText);
  biasMagZLayout->addWidget(biasMagZ);
  QHBoxLayout *magMagLayout = new QHBoxLayout;
  magMagLayout->addWidget(magMagText);
  magMagLayout->addWidget(magMag);
  QHBoxLayout *magAngLayout = new QHBoxLayout;
  magAngLayout->addWidget(magAngText);
  magAngLayout->addWidget(magAng);
  QHBoxLayout *biasGyroXLayout = new QHBoxLayout;
  biasGyroXLayout->addWidget(biasGyroXText);
  biasGyroXLayout->addWidget(biasGyroX);
  QHBoxLayout *biasGyroYLayout = new QHBoxLayout;
  biasGyroYLayout->addWidget(biasGyroYText);
  biasGyroYLayout->addWidget(biasGyroY);
  QHBoxLayout *biasGyroZLayout = new QHBoxLayout;
  biasGyroZLayout->addWidget(biasGyroZText);
  biasGyroZLayout->addWidget(biasGyroZ);
  QHBoxLayout *gyroRateLayout = new QHBoxLayout;
  gyroRateLayout->addWidget(gyroRateText);
  gyroRateLayout->addWidget(gyroRate);

  QVBoxLayout *ctrlLayout = new QVBoxLayout; 
  ctrlLayout->addLayout(acclLayout); 
  ctrlLayout->addLayout(magLayout);
  ctrlLayout->addLayout(gyroLayout);
  ctrlLayout->addLayout(imuLayout);
  ctrlLayout->addLayout(disableLayout);
  ctrlLayout->addLayout(refAcclLayout);
  ctrlLayout->addLayout(buttonLayout);
  ctrlLayout->addLayout(calibLayout);
  QVBoxLayout *paramsLayout = new QVBoxLayout;
  paramsLayout->addLayout(acclRateLayout);
  paramsLayout->addLayout(magRateLayout);
  paramsLayout->addLayout(acclAlphaLayout);
  paramsLayout->addLayout(zetaLayout);
  paramsLayout->addLayout(yawLayout);
  paramsLayout->addLayout(pitchLayout);
  paramsLayout->addLayout(rollLayout);
  paramsLayout->addLayout(acclXLayout);
  paramsLayout->addLayout(acclYLayout);
  paramsLayout->addLayout(acclZLayout);
  paramsLayout->addLayout(deltaGravLayout);
  paramsLayout->addLayout(deltaNormLayout);
  paramsLayout->addLayout(deltaAngLayout);
  paramsLayout->addLayout(deltaAcclLayout);
  paramsLayout->addLayout(deltaMagLayout);
  QVBoxLayout *biasLayout = new QVBoxLayout;
  biasLayout->addLayout(biasAcclXLayout);
  biasLayout->addLayout(biasAcclYLayout);
  biasLayout->addLayout(biasAcclZLayout);
  biasLayout->addLayout(acclMagLayout);
  biasLayout->addLayout(biasMagXLayout);
  biasLayout->addLayout(biasMagYLayout);
  biasLayout->addLayout(biasMagZLayout);
  biasLayout->addLayout(magMagLayout);
  biasLayout->addLayout(magAngLayout);
  biasLayout->addLayout(biasGyroXLayout);
  biasLayout->addLayout(biasGyroYLayout);
  biasLayout->addLayout(biasGyroZLayout);
  biasLayout->addLayout(gyroRateLayout);
  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(glWidget);
  mainLayout->addWidget(xSlider);
  mainLayout->addWidget(ySlider);
  mainLayout->addWidget(zSlider);
  mainLayout->addLayout(ctrlLayout);
  mainLayout->addSpacerItem(new QSpacerItem(10,1));
  mainLayout->addLayout(paramsLayout);
  mainLayout->addSpacerItem(new QSpacerItem(10,1));
  mainLayout->addLayout(biasLayout);
  if (video_path != NULL) {
    QVBoxLayout *windowLayout = new QVBoxLayout;
    windowLayout->addLayout(mainLayout);
    windowLayout->addWidget(video);
    setLayout(windowLayout);
    printf("new layout...\n");
  } else {
    setLayout(mainLayout);
  }

  xSlider->setValue(15 * 16);
  ySlider->setValue(345 * 16);
  zSlider->setValue(0 * 16);
  setWindowTitle(tr("Real-Time Display Tool"));
}


void Window::setRefAccl(float* a)
{
  refAcclX->setText(QString::number(a[0]));
  refAcclY->setText(QString::number(a[1]));
  refAcclZ->setText(QString::number(a[2]));
  updateRefAccl(); 
}


void Window::setParams(float* params)
{
  biasAcclX->setText(QString::number(params[0]));
  biasAcclY->setText(QString::number(params[1]));
  biasAcclZ->setText(QString::number(params[2]));
  acclMag->setText(QString::number(params[3]));
  biasMagX->setText(QString::number(params[4]));
  biasMagY->setText(QString::number(params[5]));
  biasMagZ->setText(QString::number(params[6]));
  magMag->setText(QString::number(params[7]));
  magAng->setText(QString::number(params[8]));
  biasGyroX->setText(QString::number(params[9]));
  biasGyroY->setText(QString::number(params[10]));
  biasGyroZ->setText(QString::number(params[11]));
  gyroRate->setText(QString::number(params[12])); 
  updateStateIMU();
}


void Window::updateDebug()
{
  int i;
  displayIMU_metrics* metrics;
  if (is_csv_file == 0) {
    i = sensor_buffer_index;
    yaw->setText(QString::number(sensor_buffer[i][9]));
    pitch->setText(QString::number(sensor_buffer[i][10]));
    roll->setText(QString::number(sensor_buffer[i][11]));
    acclX->setText(QString::number(sensor_buffer[i][12]));
    acclY->setText(QString::number(sensor_buffer[i][13]));
    acclZ->setText(QString::number(sensor_buffer[i][14]));
    metrics = &sensor_buffer_metrics;
  } else {
    i = csv_buffer_index;
    yaw->setText(QString::number(csv_buffer[i][9]));
    pitch->setText(QString::number(csv_buffer[i][10]));
    roll->setText(QString::number(csv_buffer[i][11]));
    acclX->setText(QString::number(csv_buffer[i][12]));
    acclY->setText(QString::number(csv_buffer[i][13]));
    acclZ->setText(QString::number(csv_buffer[i][14]));
    metrics = &csv_buffer_metrics;
  }
  deltaGrav->setText(QString::number(metrics->delta_G));
  deltaNorm->setText(QString::number(metrics->delta_M));
  deltaAng->setText(QString::number(metrics->delta_ang));
  deltaAccl->setText(QString::number(metrics->delta_a));
  deltaMag->setText(QString::number(metrics->delta_m));
}


void Window::updateCheckAccl()
{
  if (acclCheckBox->isChecked() == true)
    isIMU->setChecked(false);
  updateStateIMU();
}


void Window::updateCheckMag()
{
  if (magCheckBox->isChecked() == true)
    isIMU->setChecked(false);
  updateStateIMU();
}


void Window::updateCheckGyro()
{
  if (gyroCheckBox->isChecked() == true)
    isIMU->setChecked(false);
  updateStateIMU();
}


void Window::updateCheckIMU()
{
  if (isIMU->isChecked() == true) {
    acclCheckBox->setChecked(false);
    magCheckBox->setChecked(false);
    gyroCheckBox->setChecked(false);
  }
  updateStateIMU();
}


void Window::updateStateIMU()
{
  glWidget->isIMU         = isIMU->isChecked();
  QString textIMU         = isIMUScale->text();
  glWidget->scaleIMU      = textIMU.toFloat(); 
  glWidget->isAccl        = acclCheckBox->isChecked(); 
  QString textAccl        = acclScale->text(); 
  glWidget->scaleAccl     = textAccl.toFloat();
  glWidget->isMag         = magCheckBox->isChecked(); 
  QString textMag         = magScale->text(); 
  glWidget->scaleMag      = textMag.toFloat();
  glWidget->isGyro        = gyroCheckBox->isChecked(); 
  QString textGyro        = gyroScale->text(); 
  glWidget->scaleGyro     = textGyro.toFloat();
  if (isAccl->isChecked())
    config->isAccl        = 1;
  else
    config->isAccl        = 0;
  if (isMag->isChecked())
    config->isMagn        = 1;
  else
    config->isMagn        = 0;
  if (isGyro->isChecked())
    config->isGyro        = 1;
  else
    config->isGyro        = 0;
  QString textAcclRate    = acclRate->text();
  config->aWeight         = textAcclRate.toFloat();
  QString textMagRate     = magRate->text();
  config->mWeight         = textMagRate.toFloat();
  QString textAcclAlpha   = acclAlpha->text(); 
  config->acclAlpha       = textAcclAlpha.toFloat();
  QString textZeta        = zetaValue->text(); 
  config->autocalAlpha1   = textZeta.toFloat();
  QString textBiasAcclX   = biasAcclX->text(); 
  calib->aBias[0]         = textBiasAcclX.toFloat();
  QString textBiasAcclY   = biasAcclY->text(); 
  calib->aBias[1]         = textBiasAcclY.toFloat();
  QString textBiasAcclZ   = biasAcclZ->text(); 
  calib->aBias[2]         = textBiasAcclZ.toFloat();
  QString textAcclMag     = acclMag->text(); 
  calib->aMag             = textAcclMag.toFloat();
  QString textBiasMagX    = biasMagX->text(); 
  calib->mBias[0]         = textBiasMagX.toFloat();
  QString textBiasMagY    = biasMagY->text(); 
  calib->mBias[1]         = textBiasMagY.toFloat();
  QString textBiasMagZ    = biasMagZ->text(); 
  calib->mBias[2]         = textBiasMagZ.toFloat();
  QString textMagMag      = magMag->text();
  calib->mMag             = textMagMag.toFloat();
  QString textMagAng      = magAng->text();
  calib->mAng             = textMagAng.toFloat();
  QString textBiasGyroX   = biasGyroX->text(); 
  calib->gBias[0]         = textBiasGyroX.toFloat();
  QString textBiasGyroY   = biasGyroY->text(); 
  calib->gBias[1]         = textBiasGyroY.toFloat();
  QString textBiasGyroZ   = biasGyroZ->text(); 
  calib->gBias[2]         = textBiasGyroZ.toFloat();
  QString textGyroRate    = gyroRate->text();
  calib->gScale[0]        = textGyroRate.toFloat();
  calib->gScale[1]        = textGyroRate.toFloat();
  calib->gScale[2]        = textGyroRate.toFloat();
}


void Window::updateViewUp()
{  
  glWidget->xRot          = 90 * 16;
  glWidget->yRot          = 0;
  glWidget->zRot          = 0;
  xSlider->setValue((int)glWidget->xRot);
  ySlider->setValue((int)glWidget->yRot);
  zSlider->setValue((int)glWidget->zRot);
}
  

void Window::updateViewSide1()
{  
  glWidget->xRot       = 0;
  glWidget->yRot       = 0;
  glWidget->zRot       = 0;
  xSlider->setValue((int)glWidget->xRot);
  ySlider->setValue((int)glWidget->yRot);
  zSlider->setValue((int)glWidget->zRot);
}
  

void Window::updateViewSide2()
{  
  glWidget->xRot       = 0;
  glWidget->yRot       = 90 * 16;
  glWidget->zRot       = 0;
  xSlider->setValue((int)glWidget->xRot);
  ySlider->setValue((int)glWidget->yRot);
  zSlider->setValue((int)glWidget->zRot);
}


void Window::updateReference()
{
  sensor_IMU_set_ref   = 1;
  csv_IMU_set_ref      = 1;
}


void Window::updateResetIMU()
{
  sensor_IMU_reset     = 1;
  csv_IMU_reset        = 1;
}


void Window::updateCalibIMU()
{
  sensor_IMU_calib     = 1;
  csv_IMU_calib        = 1;
}


void Window::updateRefAccl()
{
  float refAcclVector[3];
  QString textRefAcclX    = refAcclX->text();
  refAcclVector[0]        = textRefAcclX.toFloat();
  QString textRefAcclY    = refAcclY->text();
  refAcclVector[1]        = textRefAcclY.toFloat();
  QString textRefAcclZ    = refAcclZ->text();
  refAcclVector[2]        = textRefAcclZ.toFloat();
  displayIMU_setRefAccl(refAcclVector);
}
  

QSlider *Window::createSlider()
{
  QSlider *slider = new QSlider(Qt::Vertical);
  slider->setRange(0, 360 * 16);
  slider->setSingleStep(16);
  slider->setPageStep(15 * 16);
  slider->setTickInterval(15 * 16);
  slider->setTickPosition(QSlider::TicksRight);
  return slider;
}


void Window::keyPressEvent(QKeyEvent *e)
{
  if (e->key() == Qt::Key_Escape)
    close();
  else
    QWidget::keyPressEvent(e);
}

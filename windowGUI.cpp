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

// include statements 
#include "windowGUI.h"
#include "ui_windowGUI.h"
#include "fileUtils.h"
#include <stdio.h>


/******************************************************************************
* constructor - iniatialize IMU structures and GUI
******************************************************************************/

windowGUI::windowGUI(char* config_file, char* calib_file) 
{
  // create and place window widgets
  ui = new Ui::windowGUI;
  ui->setupUi(this);

  // get pointers to IMU structures
  displayIMU_getConfig(&config);
  displayIMU_getCalib(&calib);

  // populate config structures
  //if (config_file != NULL)   
  //  displayIMU_readConfig(config_file, config);
  if (calib_file  != NULL) {
    displayIMU_readCalib(calib_file, calib);
    config_write();
  }
}


/******************************************************************************
* deconstructor - close handles and free memeory
******************************************************************************/

windowGUI::~windowGUI()
{
  delete ui;
}


/******************************************************************************
* utility function - writes contents of config structure to GUI 
******************************************************************************/

void windowGUI::config_write(void)
{
  ui->gBias0->setText(QString::number(calib->gBias[0], 'f', 2));
  ui->gBias1->setText(QString::number(calib->gBias[1], 'f', 2));
  ui->gBias2->setText(QString::number(calib->gBias[2], 'f', 2));
  ui->gMult0->setText(QString::number(calib->gMult[0], 'f', 2));
  ui->gMult1->setText(QString::number(calib->gMult[1], 'f', 2));
  ui->gMult2->setText(QString::number(calib->gMult[2], 'f', 2));
  ui->gMult3->setText(QString::number(calib->gMult[3], 'f', 2));
  ui->gMult4->setText(QString::number(calib->gMult[4], 'f', 2));
  ui->gMult5->setText(QString::number(calib->gMult[5], 'f', 2));
  ui->gMult6->setText(QString::number(calib->gMult[6], 'f', 2));
  ui->gMult7->setText(QString::number(calib->gMult[7], 'f', 2));
  ui->gMult8->setText(QString::number(calib->gMult[8], 'f', 2));
  ui->aBias0->setText(QString::number(calib->aBias[0], 'f', 2));
  ui->aBias1->setText(QString::number(calib->aBias[1], 'f', 2));
  ui->aBias2->setText(QString::number(calib->aBias[2], 'f', 2));
  ui->aMult0->setText(QString::number(calib->aMult[0], 'f', 2));
  ui->aMult1->setText(QString::number(calib->aMult[1], 'f', 2));
  ui->aMult2->setText(QString::number(calib->aMult[2], 'f', 2));
  ui->aMult3->setText(QString::number(calib->aMult[3], 'f', 2));
  ui->aMult4->setText(QString::number(calib->aMult[4], 'f', 2));
  ui->aMult5->setText(QString::number(calib->aMult[5], 'f', 2));
  ui->aMult6->setText(QString::number(calib->aMult[6], 'f', 2));
  ui->aMult7->setText(QString::number(calib->aMult[7], 'f', 2));
  ui->aMult8->setText(QString::number(calib->aMult[8], 'f', 2));
  ui->mBias0->setText(QString::number(calib->mBias[0], 'f', 2));
  ui->mBias1->setText(QString::number(calib->mBias[1], 'f', 2));
  ui->mBias2->setText(QString::number(calib->mBias[2], 'f', 2));
  ui->mMult0->setText(QString::number(calib->mMult[0], 'f', 2));
  ui->mMult1->setText(QString::number(calib->mMult[1], 'f', 2));
  ui->mMult2->setText(QString::number(calib->mMult[2], 'f', 2));
  ui->mMult3->setText(QString::number(calib->mMult[3], 'f', 2));
  ui->mMult4->setText(QString::number(calib->mMult[4], 'f', 2));
  ui->mMult5->setText(QString::number(calib->mMult[5], 'f', 2));
  ui->mMult6->setText(QString::number(calib->mMult[6], 'f', 2));
  ui->mMult7->setText(QString::number(calib->mMult[7], 'f', 2));
  ui->mMult8->setText(QString::number(calib->mMult[8], 'f', 2));
  ui->aMag->setText(QString::number(calib->aMag, 'f', 2));
  ui->gMag->setText(QString::number(calib->mMag, 'f', 2));
  ui->gAng->setText(QString::number(calib->mAng, 'f', 2));
}

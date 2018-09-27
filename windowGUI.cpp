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
#include <QFileDialog>
#include "windowGUI.h"
#include "ui_windowGUI.h"
#include "fileUtils.h"


/******************************************************************************
* constructor - create GUI and allocate memory 
******************************************************************************/

windowGUI::windowGUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::windowGUI)
{
  // create and place window widgets
  ui->setupUi(this);

  // get pointers to IMU structures
  displayIMU_getConfig(&config);
  displayIMU_getCalib(&calib);
}


/******************************************************************************
* iniatialize IMU structures and GUI elements 
******************************************************************************/

void windowGUI::initIMU(char* config_file, char* calib_file)
{
  if (config_file != NULL) {
    displayIMU_readConfig(config_file, config);
    config_write();
  }
  if (calib_file  != NULL) {
    displayIMU_readCalib(calib_file, calib);
    calib_write();
  }
}


/******************************************************************************
* utility function - writes contents of config structure to GUI 
******************************************************************************/

void windowGUI::config_write()
{
  ui->noGyro->setChecked(!config->isGyro);
  ui->noAccl->setChecked(!config->isAccl);
  ui->noMagn->setChecked(!config->isMagn);
  ui->noFltr->setChecked(!config->isFltr);
  ui->noTear->setChecked(!config->isTear);
  ui->noMove->setChecked(!config->isMove);
  ui->noFOM->setChecked(!config->isFOM);
  ui->noAutocal->setChecked(!config->isAutocal);
  ui->gThreshVal->setText(QString::number(config->gThreshVal, 'f', 2));
  ui->gThreshTime->setText(QString::number(config->gThreshTime, 'f', 2));
  ui->aWeight->setText(QString::number(config->aWeight, 'f', 2));
  ui->aAlpha->setText(QString::number(config->aAlpha, 'f', 2));
  ui->mWeight->setText(QString::number(config->mWeight, 'f', 2));
  ui->mAlpha->setText(QString::number(config->mAlpha, 'f', 2));
  ui->moveAlpha->setText(QString::number(config->moveAlpha, 'f', 2));
  ui->autocalAlpha->setText(QString::number(config->autocalAlpha, 'f', 2)); 
}


/******************************************************************************
* utility function - read contents of config structure to GUI 
******************************************************************************/

void windowGUI::config_read()
{
  config->isGyro       = !ui->noGyro->isChecked();
  config->isAccl       = !ui->noAccl->isChecked();
  config->isMagn       = !ui->noMagn->isChecked();
  config->isFltr       = !ui->noFltr->isChecked();
  config->isTear       = !ui->noTear->isChecked();
  config->isMove       = !ui->noMove->isChecked();
  config->isFOM        = !ui->noFOM->isChecked();
  config->gThreshVal   = ui->gThreshVal->text().toFloat();
  config->gThreshTime  = ui->gThreshTime->text().toFloat();
  config->aWeight      = ui->aWeight->text().toFloat();
  config->aAlpha       = ui->aAlpha->text().toFloat();
  config->mWeight      = ui->mWeight->text().toFloat();
  config->mAlpha       = ui->mAlpha->text().toFloat();
  config->moveAlpha    = ui->moveAlpha->text().toFloat();
  config->autocalAlpha = ui->autocalAlpha->text().toFloat();
}


/******************************************************************************
* deconstructor - close handles and free memeory
******************************************************************************/

windowGUI::~windowGUI()
{
  delete ui;
}


/******************************************************************************
* utility function - writes contents of calib structure to GUI 
******************************************************************************/

void windowGUI::calib_write()
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
  ui->mMag->setText(QString::number(calib->mMag, 'f', 2));
  ui->mAng->setText(QString::number(calib->mAng, 'f', 2));
}


/******************************************************************************
* utility function - read contents of calib structure to GUI 
******************************************************************************/

void windowGUI::calib_read()
{
  calib->gBias[0] = ui->gBias0->text().toFloat();
  calib->gBias[1] = ui->gBias1->text().toFloat();
  calib->gBias[2] = ui->gBias2->text().toFloat();
  calib->gMult[0] = ui->gMult0->text().toFloat();
  calib->gMult[1] = ui->gMult1->text().toFloat();
  calib->gMult[2] = ui->gMult2->text().toFloat();
  calib->gMult[3] = ui->gMult3->text().toFloat();
  calib->gMult[4] = ui->gMult4->text().toFloat();
  calib->gMult[5] = ui->gMult5->text().toFloat();
  calib->gMult[6] = ui->gMult6->text().toFloat();
  calib->gMult[7] = ui->gMult7->text().toFloat();
  calib->gMult[8] = ui->gMult8->text().toFloat();
  calib->aBias[0] = ui->aBias0->text().toFloat();
  calib->aBias[1] = ui->aBias1->text().toFloat();
  calib->aBias[2] = ui->aBias2->text().toFloat();
  calib->aMult[0] = ui->aMult0->text().toFloat();
  calib->aMult[1] = ui->aMult1->text().toFloat();
  calib->aMult[2] = ui->aMult2->text().toFloat();
  calib->aMult[3] = ui->aMult3->text().toFloat();
  calib->aMult[4] = ui->aMult4->text().toFloat();
  calib->aMult[5] = ui->aMult5->text().toFloat();
  calib->aMult[6] = ui->aMult6->text().toFloat();
  calib->aMult[7] = ui->aMult7->text().toFloat();
  calib->aMult[8] = ui->aMult8->text().toFloat();
  calib->mBias[0] = ui->mBias0->text().toFloat();
  calib->mBias[1] = ui->mBias1->text().toFloat();
  calib->mBias[2] = ui->mBias2->text().toFloat();
  calib->mMult[0] = ui->mMult0->text().toFloat();
  calib->mMult[1] = ui->mMult1->text().toFloat();
  calib->mMult[2] = ui->mMult2->text().toFloat();
  calib->mMult[3] = ui->mMult3->text().toFloat();
  calib->mMult[4] = ui->mMult4->text().toFloat();
  calib->mMult[5] = ui->mMult5->text().toFloat();
  calib->mMult[6] = ui->mMult6->text().toFloat();
  calib->mMult[7] = ui->mMult7->text().toFloat();
  calib->mMult[8] = ui->mMult8->text().toFloat();
  calib->aMag     = ui->aMag->text().toFloat();
  calib->mMag     = ui->mMag->text().toFloat();
  calib->mAng     = ui->mAng->text().toFloat();
}


/******************************************************************************
* loads config structure from json file
******************************************************************************/

void windowGUI::on_configOpen_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), ".",
    ("json (*.json)"));
  displayIMU_readConfig((char *)file.toStdString().c_str(), config);
  config_write();
}


/******************************************************************************
* saves config structure to json file
******************************************************************************/

void windowGUI::on_configSave_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), ".",
    ("json (*.json)"));
  displayIMU_writeConfig((char *)file.toStdString().c_str(), config);
}


/******************************************************************************
* loads calib structure from json file
******************************************************************************/

void windowGUI::on_calibOpen_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), ".", 
    ("json (*.json)"));
  displayIMU_readCalib((char *)file.toStdString().c_str(), calib);
  calib_write();
}


/******************************************************************************
* saves calib structure to json file
******************************************************************************/

void windowGUI::on_calibSave_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), ".",
    ("json (*.json)"));
  displayIMU_writeCalib((char *)file.toStdString().c_str(), calib);
}


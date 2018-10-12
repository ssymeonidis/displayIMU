/*
 * This file is part of quaternion-based displayIMU C/C++/QT code base
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
#include "IMU_file.h"
#include "windowGUI.h"
#include "ui_windowGUI.h"


/******************************************************************************
* constructor - create GUI and allocate memory 
******************************************************************************/

windowGUI::windowGUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::windowGUI)
{
  // create and place window widgets
  ui->setupUi(this);

  // get pointers to config and state structures
  IMU_union_config configUnion;
  IMU_engn_getConfig(0, IMU_engn_core, &configUnion);
  configCore = configUnion.configCore;
  IMU_engn_getConfig(0, IMU_engn_rect, &configUnion);
  configRect = configUnion.configRect;

  // initialize display/sensor IF parameters
  load_json((char *)"../config/displayIMU.json");
  glWidget_update();
}


/******************************************************************************
* deconstructor - close handles and free memeory
******************************************************************************/

windowGUI::~windowGUI()
{
  delete ui;
}


/******************************************************************************
* iniatialize IMU structures and GUI elements 
******************************************************************************/

void windowGUI::initIMU(char* core_config_file, char* rect_config_file)
{
  if (core_config_file != NULL) {
    IMU_engn_load(0, core_config_file, IMU_engn_core);
    config_write();
  }
  if (rect_config_file  != NULL) {
    IMU_engn_load(0, rect_config_file, IMU_engn_rect);
    calib_write();
  }
}


/******************************************************************************
* utility function - writes contents of config structure to GUI 
******************************************************************************/

void windowGUI::config_write()
{
  ui->noGyro->setChecked(!configCore->isGyro);
  ui->noAccl->setChecked(!configCore->isAccl);
  ui->noMagn->setChecked(!configCore->isMagn);
  ui->noStable->setChecked(!configCore->isStable);
  ui->noFOM->setChecked(!configCore->isFOM);
  ui->noMove->setChecked(!configCore->isMove);
  ui->gThresh->setText(QString::number(configCore->gThresh, 'f', 2));
  ui->gThreshTime->setText(QString::number(configCore->gThreshTime, 'f', 2));
  ui->aWeight->setText(QString::number(configCore->aWeight, 'f', 2));
  ui->aMag->setText(QString::number(configCore->aMag, 'f', 2));
  ui->aMagThresh->setText(QString::number(configCore->aMagThresh, 'f', 2));
  ui->mWeight->setText(QString::number(configCore->mWeight, 'f', 2));
  ui->mMag->setText(QString::number(configCore->mMag, 'f', 2));
  ui->mMagThresh->setText(QString::number(configCore->mMagThresh, 'f', 2));
  ui->mAng->setText(QString::number(configCore->mAng, 'f', 2));
  ui->mAngThresh->setText(QString::number(configCore->mAngThresh, 'f', 2));
  ui->moveAlpha->setText(QString::number(configCore->moveAlpha, 'f', 2));
}


/******************************************************************************
* utility function - read contents of config structure to GUI 
******************************************************************************/

void windowGUI::config_read()
{
  configCore->isGyro       = !ui->noGyro->isChecked();
  configCore->isAccl       = !ui->noAccl->isChecked();
  configCore->isMagn       = !ui->noMagn->isChecked();
  configCore->isStable     = !ui->noStable->isChecked();
  configCore->isFOM        = !ui->noFOM->isChecked();
  configCore->isMove       = !ui->noMove->isChecked();
  configCore->gThresh      = ui->gThresh->text().toFloat();
  configCore->gThreshTime  = ui->gThreshTime->text().toFloat();
  configCore->aWeight      = ui->aWeight->text().toFloat();
  configCore->aMag         = ui->aMag->text().toFloat();
  configCore->aMagThresh   = ui->aMagThresh->text().toFloat();
  configCore->mWeight      = ui->mWeight->text().toFloat();
  configCore->mMag         = ui->mMag->text().toFloat();
  configCore->mMagThresh   = ui->mMagThresh->text().toFloat();
  configCore->mAng         = ui->mAng->text().toFloat();
  configCore->mAngThresh   = ui->mAngThresh->text().toFloat();
  configCore->moveAlpha    = ui->moveAlpha->text().toFloat();
}


/******************************************************************************
* utility function - writes contents of calib structure to GUI 
******************************************************************************/

void windowGUI::calib_write()
{
  ui->gBias0->setText(QString::number(configRect->gBias[0], 'f', 2));
  ui->gBias1->setText(QString::number(configRect->gBias[1], 'f', 2));
  ui->gBias2->setText(QString::number(configRect->gBias[2], 'f', 2));
  ui->gMult0->setText(QString::number(configRect->gMult[0], 'f', 2));
  ui->gMult1->setText(QString::number(configRect->gMult[1], 'f', 2));
  ui->gMult2->setText(QString::number(configRect->gMult[2], 'f', 2));
  ui->gMult3->setText(QString::number(configRect->gMult[3], 'f', 2));
  ui->gMult4->setText(QString::number(configRect->gMult[4], 'f', 2));
  ui->gMult5->setText(QString::number(configRect->gMult[5], 'f', 2));
  ui->gMult6->setText(QString::number(configRect->gMult[6], 'f', 2));
  ui->gMult7->setText(QString::number(configRect->gMult[7], 'f', 2));
  ui->gMult8->setText(QString::number(configRect->gMult[8], 'f', 2));
  ui->aBias0->setText(QString::number(configRect->aBias[0], 'f', 2));
  ui->aBias1->setText(QString::number(configRect->aBias[1], 'f', 2));
  ui->aBias2->setText(QString::number(configRect->aBias[2], 'f', 2));
  ui->aMult0->setText(QString::number(configRect->aMult[0], 'f', 2));
  ui->aMult1->setText(QString::number(configRect->aMult[1], 'f', 2));
  ui->aMult2->setText(QString::number(configRect->aMult[2], 'f', 2));
  ui->aMult3->setText(QString::number(configRect->aMult[3], 'f', 2));
  ui->aMult4->setText(QString::number(configRect->aMult[4], 'f', 2));
  ui->aMult5->setText(QString::number(configRect->aMult[5], 'f', 2));
  ui->aMult6->setText(QString::number(configRect->aMult[6], 'f', 2));
  ui->aMult7->setText(QString::number(configRect->aMult[7], 'f', 2));
  ui->aMult8->setText(QString::number(configRect->aMult[8], 'f', 2));
  ui->mBias0->setText(QString::number(configRect->mBias[0], 'f', 2));
  ui->mBias1->setText(QString::number(configRect->mBias[1], 'f', 2));
  ui->mBias2->setText(QString::number(configRect->mBias[2], 'f', 2));
  ui->mMult0->setText(QString::number(configRect->mMult[0], 'f', 2));
  ui->mMult1->setText(QString::number(configRect->mMult[1], 'f', 2));
  ui->mMult2->setText(QString::number(configRect->mMult[2], 'f', 2));
  ui->mMult3->setText(QString::number(configRect->mMult[3], 'f', 2));
  ui->mMult4->setText(QString::number(configRect->mMult[4], 'f', 2));
  ui->mMult5->setText(QString::number(configRect->mMult[5], 'f', 2));
  ui->mMult6->setText(QString::number(configRect->mMult[6], 'f', 2));
  ui->mMult7->setText(QString::number(configRect->mMult[7], 'f', 2));
  ui->mMult8->setText(QString::number(configRect->mMult[8], 'f', 2));
}


/******************************************************************************
* utility function - read contents of calib structure to GUI 
******************************************************************************/

void windowGUI::calib_read()
{
  configRect->gBias[0] = ui->gBias0->text().toFloat();
  configRect->gBias[1] = ui->gBias1->text().toFloat();
  configRect->gBias[2] = ui->gBias2->text().toFloat();
  configRect->gMult[0] = ui->gMult0->text().toFloat();
  configRect->gMult[1] = ui->gMult1->text().toFloat();
  configRect->gMult[2] = ui->gMult2->text().toFloat();
  configRect->gMult[3] = ui->gMult3->text().toFloat();
  configRect->gMult[4] = ui->gMult4->text().toFloat();
  configRect->gMult[5] = ui->gMult5->text().toFloat();
  configRect->gMult[6] = ui->gMult6->text().toFloat();
  configRect->gMult[7] = ui->gMult7->text().toFloat();
  configRect->gMult[8] = ui->gMult8->text().toFloat();
  configRect->aBias[0] = ui->aBias0->text().toFloat();
  configRect->aBias[1] = ui->aBias1->text().toFloat();
  configRect->aBias[2] = ui->aBias2->text().toFloat();
  configRect->aMult[0] = ui->aMult0->text().toFloat();
  configRect->aMult[1] = ui->aMult1->text().toFloat();
  configRect->aMult[2] = ui->aMult2->text().toFloat();
  configRect->aMult[3] = ui->aMult3->text().toFloat();
  configRect->aMult[4] = ui->aMult4->text().toFloat();
  configRect->aMult[5] = ui->aMult5->text().toFloat();
  configRect->aMult[6] = ui->aMult6->text().toFloat();
  configRect->aMult[7] = ui->aMult7->text().toFloat();
  configRect->aMult[8] = ui->aMult8->text().toFloat();
  configRect->mBias[0] = ui->mBias0->text().toFloat();
  configRect->mBias[1] = ui->mBias1->text().toFloat();
  configRect->mBias[2] = ui->mBias2->text().toFloat();
  configRect->mMult[0] = ui->mMult0->text().toFloat();
  configRect->mMult[1] = ui->mMult1->text().toFloat();
  configRect->mMult[2] = ui->mMult2->text().toFloat();
  configRect->mMult[3] = ui->mMult3->text().toFloat();
  configRect->mMult[4] = ui->mMult4->text().toFloat();
  configRect->mMult[5] = ui->mMult5->text().toFloat();
  configRect->mMult[6] = ui->mMult6->text().toFloat();
  configRect->mMult[7] = ui->mMult7->text().toFloat();
  configRect->mMult[8] = ui->mMult8->text().toFloat();
}


/******************************************************************************
* saves calib structure to json file
******************************************************************************/

void windowGUI::load_json(char* filename)
{
  // define internal variables
  FILE*     file;
  char*     field;
  char*     args;
  float     val;
  int       status;

  // open json file containing config struct
  file = fopen(filename, "r");
  if (file == NULL)
    return;

  // main loop that parse json file line by line
  while (1) {

    // read line and parse field/args
    status = IMU_file_getLine(file, &field, &args);
    if (status > 1 || status < 0)
      break;

    // extract arguments for the specified field
    if        (strcmp(field, "gyro") == 0) {
      sscanf(args, "%f", &val);
      ui->dispScaleGyro->setText(QString::number(val, 'f', 1));
    } else if (strcmp(field, "accl") == 0) {
      sscanf(args, "%f", &val);
      ui->dispScaleAccl->setText(QString::number(val, 'f', 1));
    } else if (strcmp(field, "magn") == 0) {
      sscanf(args, "%f", &val);
      ui->dispScaleMagn->setText(QString::number(val, 'f', 1));
    } else if (strcmp(field, "IMU") == 0) {
      sscanf(args, "%f", &val);
      ui->dispScaleIMU->setText(QString::number(val, 'f', 1));
    }
  }

  // exit function
  fclose(file);
}


/******************************************************************************
* updates display scale for gyroscope
******************************************************************************/

void windowGUI::glWidget_update()
{
  ui->widget->scaleGyro = ui->dispScaleGyro->text().toFloat();
  ui->widget->scaleAccl = ui->dispScaleAccl->text().toFloat();
  ui->widget->scaleMagn = ui->dispScaleMagn->text().toFloat();
  ui->widget->scaleIMU  = ui->dispScaleIMU->text().toFloat();
  ui->widget->isGyro    = ui->dispEnableGyro->isChecked();
  ui->widget->isAccl    = ui->dispEnableAccl->isChecked();
  ui->widget->isMagn    = ui->dispEnableMagn->isChecked();
  ui->widget->isIMU     = ui->dispEnableIMU->isChecked();
}


/******************************************************************************
* loads config structure from json file
******************************************************************************/

void windowGUI::on_configOpen_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), "../config",
    ("json (*.json)"));
  IMU_engn_load(0, (char *)file.toStdString().c_str(), IMU_engn_core);
  config_write();
}


/******************************************************************************
* saves config structure to json file
******************************************************************************/

void windowGUI::on_configSave_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), "../config",
    ("json (*.json)"));
  IMU_engn_save(0, (char *)file.toStdString().c_str(), IMU_engn_core);
}


/******************************************************************************
* loads calib structure from json file
******************************************************************************/

void windowGUI::on_calibOpen_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), "../config", 
    ("json (*.json)"));
  IMU_engn_load(0, (char *)file.toStdString().c_str(), IMU_engn_rect);
  calib_write();
}


/******************************************************************************
* saves calib structure to json file
******************************************************************************/

void windowGUI::on_calibSave_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), "../config",
    ("json (*.json)"));
  IMU_engn_save(0, (char *)file.toStdString().c_str(), IMU_engn_rect);
}


/******************************************************************************
* enables display of gyroscope data
******************************************************************************/

void windowGUI::on_dispEnableGyro_clicked()
{
  ui->dispEnableIMU->setChecked(false);
  glWidget_update();
}


/******************************************************************************
* enables display of accelerometer data
******************************************************************************/

void windowGUI::on_dispEnableAccl_clicked()
{
  ui->dispEnableIMU->setChecked(false);
  glWidget_update();
}


/******************************************************************************
* enables display of accelerometer data
******************************************************************************/

void windowGUI::on_dispEnableMagn_clicked()
{
  ui->dispEnableIMU->setChecked(false);
  glWidget_update();
}


/******************************************************************************
* enables display of IMU results
******************************************************************************/

void windowGUI::on_dispEnableIMU_clicked()
{
  ui->dispEnableGyro->setChecked(false);
  ui->dispEnableAccl->setChecked(false);
  ui->dispEnableMagn->setChecked(false);
  glWidget_update();
}


/******************************************************************************
* change to top-down view
******************************************************************************/

void windowGUI::on_viewUp_clicked()
{
  ui->widget->xRot     = 90 * 16;
  ui->widget->yRot     = 0;
  ui->widget->zRot     = 0;
}


/******************************************************************************
* change to side view
******************************************************************************/

void windowGUI::on_viewSide1_clicked()
{
  ui->widget->xRot     = 0;
  ui->widget->yRot     = 0;
  ui->widget->zRot     = 0;
}


/******************************************************************************
* change to side view
******************************************************************************/

void windowGUI::on_viewSide2_clicked()
{
  ui->widget->xRot     = 0;
  ui->widget->yRot     = 90 * 16;
  ui->widget->zRot     = 0;
}

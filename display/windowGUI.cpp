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
#include "windowGUI.h"
#include "ui_windowGUI.h"
#include "IMU_util_file.h"


/******************************************************************************
* constructor - create GUI and allocate memory 
******************************************************************************/

windowGUI::windowGUI(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::windowGUI)
{
  // define local variables (not really used)
  unsigned short IMU_core_id; 
  unsigned short IMU_correct_id;
 
  // create and place window widgets
  ui->setupUi(this);

  // get pointers to IMU structures
  IMU_core_init(&IMU_core_id, &core_config);
  IMU_correct_init(&IMU_correct_id, &correct_config);
  
  // initialize display parameters
  load_json((char *)"../config/displayIMU.json");
  glWidget_update();
}


/******************************************************************************
* iniatialize IMU structures and GUI elements 
******************************************************************************/

void windowGUI::initIMU(char* core_config_file, char* correct_config_file)
{
  if (core_config_file != NULL) {
    IMU_util_readCore(core_config_file, core_config);
    config_write();
  }
  if (correct_config_file  != NULL) {
    IMU_util_readCorrect(correct_config_file, correct_config);
    calib_write();
  }
}


/******************************************************************************
* utility function - writes contents of config structure to GUI 
******************************************************************************/

void windowGUI::config_write()
{
  ui->noGyro->setChecked(!core_config->isGyro);
  ui->noAccl->setChecked(!core_config->isAccl);
  ui->noMagn->setChecked(!core_config->isMagn);
  ui->noStable->setChecked(!core_config->isStable);
  ui->noFOM->setChecked(!core_config->isFOM);
  ui->noMove->setChecked(!core_config->isMove);
  ui->gThreshVal->setText(QString::number(core_config->gThreshVal, 'f', 2));
  ui->gThreshTime->setText(QString::number(core_config->gThreshTime, 'f', 2));
  ui->aWeight->setText(QString::number(core_config->aWeight, 'f', 2));
  ui->aMag->setText(QString::number(core_config->aMag, 'f', 2));
  ui->aMagThresh->setText(QString::number(core_config->aMagThresh, 'f', 2));
  ui->mWeight->setText(QString::number(core_config->mWeight, 'f', 2));
  ui->mMag->setText(QString::number(core_config->mMag, 'f', 2));
  ui->mMagThresh->setText(QString::number(core_config->mMagThresh, 'f', 2));
  ui->mAng->setText(QString::number(core_config->mAng, 'f', 2));
  ui->mAngThresh->setText(QString::number(core_config->mAngThresh, 'f', 2));
  ui->moveAlpha->setText(QString::number(core_config->moveAlpha, 'f', 2));
}


/******************************************************************************
* utility function - read contents of config structure to GUI 
******************************************************************************/

void windowGUI::config_read()
{
  core_config->isGyro       = !ui->noGyro->isChecked();
  core_config->isAccl       = !ui->noAccl->isChecked();
  core_config->isMagn       = !ui->noMagn->isChecked();
  core_config->isStable     = !ui->noStable->isChecked();
  core_config->isFOM        = !ui->noFOM->isChecked();
  core_config->isMove       = !ui->noMove->isChecked();
  core_config->gThreshVal   = ui->gThreshVal->text().toFloat();
  core_config->gThreshTime  = ui->gThreshTime->text().toFloat();
  core_config->aWeight      = ui->aWeight->text().toFloat();
  core_config->aMag         = ui->aMag->text().toFloat();
  core_config->aMagThresh   = ui->aMagThresh->text().toFloat();
  core_config->mWeight      = ui->mWeight->text().toFloat();
  core_config->mMag         = ui->mMag->text().toFloat();
  core_config->mMagThresh   = ui->mMagThresh->text().toFloat();
  core_config->mAng         = ui->mAng->text().toFloat();
  core_config->mAngThresh   = ui->mAngThresh->text().toFloat();
  core_config->moveAlpha    = ui->moveAlpha->text().toFloat();
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
  ui->gBias0->setText(QString::number(correct_config->gBias[0], 'f', 2));
  ui->gBias1->setText(QString::number(correct_config->gBias[1], 'f', 2));
  ui->gBias2->setText(QString::number(correct_config->gBias[2], 'f', 2));
  ui->gMult0->setText(QString::number(correct_config->gMult[0], 'f', 2));
  ui->gMult1->setText(QString::number(correct_config->gMult[1], 'f', 2));
  ui->gMult2->setText(QString::number(correct_config->gMult[2], 'f', 2));
  ui->gMult3->setText(QString::number(correct_config->gMult[3], 'f', 2));
  ui->gMult4->setText(QString::number(correct_config->gMult[4], 'f', 2));
  ui->gMult5->setText(QString::number(correct_config->gMult[5], 'f', 2));
  ui->gMult6->setText(QString::number(correct_config->gMult[6], 'f', 2));
  ui->gMult7->setText(QString::number(correct_config->gMult[7], 'f', 2));
  ui->gMult8->setText(QString::number(correct_config->gMult[8], 'f', 2));
  ui->aBias0->setText(QString::number(correct_config->aBias[0], 'f', 2));
  ui->aBias1->setText(QString::number(correct_config->aBias[1], 'f', 2));
  ui->aBias2->setText(QString::number(correct_config->aBias[2], 'f', 2));
  ui->aMult0->setText(QString::number(correct_config->aMult[0], 'f', 2));
  ui->aMult1->setText(QString::number(correct_config->aMult[1], 'f', 2));
  ui->aMult2->setText(QString::number(correct_config->aMult[2], 'f', 2));
  ui->aMult3->setText(QString::number(correct_config->aMult[3], 'f', 2));
  ui->aMult4->setText(QString::number(correct_config->aMult[4], 'f', 2));
  ui->aMult5->setText(QString::number(correct_config->aMult[5], 'f', 2));
  ui->aMult6->setText(QString::number(correct_config->aMult[6], 'f', 2));
  ui->aMult7->setText(QString::number(correct_config->aMult[7], 'f', 2));
  ui->aMult8->setText(QString::number(correct_config->aMult[8], 'f', 2));
  ui->mBias0->setText(QString::number(correct_config->mBias[0], 'f', 2));
  ui->mBias1->setText(QString::number(correct_config->mBias[1], 'f', 2));
  ui->mBias2->setText(QString::number(correct_config->mBias[2], 'f', 2));
  ui->mMult0->setText(QString::number(correct_config->mMult[0], 'f', 2));
  ui->mMult1->setText(QString::number(correct_config->mMult[1], 'f', 2));
  ui->mMult2->setText(QString::number(correct_config->mMult[2], 'f', 2));
  ui->mMult3->setText(QString::number(correct_config->mMult[3], 'f', 2));
  ui->mMult4->setText(QString::number(correct_config->mMult[4], 'f', 2));
  ui->mMult5->setText(QString::number(correct_config->mMult[5], 'f', 2));
  ui->mMult6->setText(QString::number(correct_config->mMult[6], 'f', 2));
  ui->mMult7->setText(QString::number(correct_config->mMult[7], 'f', 2));
  ui->mMult8->setText(QString::number(correct_config->mMult[8], 'f', 2));
}


/******************************************************************************
* utility function - read contents of calib structure to GUI 
******************************************************************************/

void windowGUI::calib_read()
{
  correct_config->gBias[0] = ui->gBias0->text().toFloat();
  correct_config->gBias[1] = ui->gBias1->text().toFloat();
  correct_config->gBias[2] = ui->gBias2->text().toFloat();
  correct_config->gMult[0] = ui->gMult0->text().toFloat();
  correct_config->gMult[1] = ui->gMult1->text().toFloat();
  correct_config->gMult[2] = ui->gMult2->text().toFloat();
  correct_config->gMult[3] = ui->gMult3->text().toFloat();
  correct_config->gMult[4] = ui->gMult4->text().toFloat();
  correct_config->gMult[5] = ui->gMult5->text().toFloat();
  correct_config->gMult[6] = ui->gMult6->text().toFloat();
  correct_config->gMult[7] = ui->gMult7->text().toFloat();
  correct_config->gMult[8] = ui->gMult8->text().toFloat();
  correct_config->aBias[0] = ui->aBias0->text().toFloat();
  correct_config->aBias[1] = ui->aBias1->text().toFloat();
  correct_config->aBias[2] = ui->aBias2->text().toFloat();
  correct_config->aMult[0] = ui->aMult0->text().toFloat();
  correct_config->aMult[1] = ui->aMult1->text().toFloat();
  correct_config->aMult[2] = ui->aMult2->text().toFloat();
  correct_config->aMult[3] = ui->aMult3->text().toFloat();
  correct_config->aMult[4] = ui->aMult4->text().toFloat();
  correct_config->aMult[5] = ui->aMult5->text().toFloat();
  correct_config->aMult[6] = ui->aMult6->text().toFloat();
  correct_config->aMult[7] = ui->aMult7->text().toFloat();
  correct_config->aMult[8] = ui->aMult8->text().toFloat();
  correct_config->mBias[0] = ui->mBias0->text().toFloat();
  correct_config->mBias[1] = ui->mBias1->text().toFloat();
  correct_config->mBias[2] = ui->mBias2->text().toFloat();
  correct_config->mMult[0] = ui->mMult0->text().toFloat();
  correct_config->mMult[1] = ui->mMult1->text().toFloat();
  correct_config->mMult[2] = ui->mMult2->text().toFloat();
  correct_config->mMult[3] = ui->mMult3->text().toFloat();
  correct_config->mMult[4] = ui->mMult4->text().toFloat();
  correct_config->mMult[5] = ui->mMult5->text().toFloat();
  correct_config->mMult[6] = ui->mMult6->text().toFloat();
  correct_config->mMult[7] = ui->mMult7->text().toFloat();
  correct_config->mMult[8] = ui->mMult8->text().toFloat();
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
    status = IMU_util_getLine(file, &field, &args);
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
  IMU_util_readCore((char *)file.toStdString().c_str(), core_config);
  config_write();
}


/******************************************************************************
* saves config structure to json file
******************************************************************************/

void windowGUI::on_configSave_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), "../config",
    ("json (*.json)"));
  IMU_util_writeCore((char *)file.toStdString().c_str(), core_config);
}


/******************************************************************************
* loads calib structure from json file
******************************************************************************/

void windowGUI::on_calibOpen_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), "../config", 
    ("json (*.json)"));
  IMU_util_readCorrect((char *)file.toStdString().c_str(), correct_config);
  calib_write();
}


/******************************************************************************
* saves calib structure to json file
******************************************************************************/

void windowGUI::on_calibSave_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), "../config",
    ("json (*.json)"));
  IMU_util_writeCorrect((char *)file.toStdString().c_str(), correct_config);
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

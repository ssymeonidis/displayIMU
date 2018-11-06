/*
 * This file is part of quaternion-based displayIMU C/C++/QT code base
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
  configCore = configUnion.core;
  IMU_engn_getConfig(0, IMU_engn_rect, &configUnion);
  configRect = configUnion.rect;

  // initialize display/sensor IF parameters
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

void windowGUI::initIMU(configGUI *config)
{
  // initialize display/sensor IF parameters
  ui->disp_scaleGyro->setText(QString::number(config->gyro, 'f', 1));
  ui->disp_scaleAccl->setText(QString::number(config->accl, 'f', 1));
  ui->disp_scaleMagn->setText(QString::number(config->magn, 'f', 1));
  ui->disp_scaleIMU->setText (QString::number(config->imu, 'f', 1));
  glWidget_update();
}


/******************************************************************************
* utility function - writes contents of config structure to GUI 
******************************************************************************/

void windowGUI::config_write()
{
  ui->core_isGyro->setChecked(configCore->isGyro);
  ui->core_isAccl->setChecked(configCore->isAccl);
  ui->core_isMagn->setChecked(configCore->isMagn);
  //ui->noStable->setChecked(!configCore->isStable);
  ui->core_isFOM->setChecked(configCore->isFOM);
  //ui->noMove->setChecked(!configCore->isMove);
  //ui->gThresh->setText(QString::number(configCore->gThresh, 'f', 2));
  //ui->gThreshTime->setText(QString::number(configCore->gThreshTime, 'f', 2));
  ui->core_aWeight->setText(QString::number(configCore->aWeight, 'f', 2));
  ui->core_aMag->setText(QString::number(configCore->aMag, 'f', 2));
  ui->core_aMagThresh->setText(QString::number(configCore->aMagThresh, 'f', 2));
  ui->core_mWeight->setText(QString::number(configCore->mWeight, 'f', 2));
  ui->core_mMag->setText(QString::number(configCore->mMag, 'f', 2));
  ui->core_mMagThresh->setText(QString::number(configCore->mMagThresh, 'f', 2));
  //ui->mAng->setText(QString::number(configCore->mAng, 'f', 2));
  //ui->mAngThresh->setText(QString::number(configCore->mAngThresh, 'f', 2));
  //ui->moveAlpha->setText(QString::number(configCore->moveAlpha, 'f', 2));
}


/******************************************************************************
* utility function - read contents of config structure to GUI 
******************************************************************************/

void windowGUI::config_read()
{
  configCore->isGyro       = ui->core_isGyro->isChecked();
  configCore->isAccl       = ui->core_isAccl->isChecked();
  configCore->isMagn       = ui->core_isMagn->isChecked();
  //configCore->isStable     = !ui->noStable->isChecked();
  configCore->isFOM        = ui->core_isFOM->isChecked();
  //configCore->isMove       = !ui->noMove->isChecked();
  //configCore->gThresh      = ui->gThresh->text().toFloat();
  //configCore->gThreshTime  = ui->gThreshTime->text().toFloat();
  configCore->aWeight      = ui->core_aWeight->text().toFloat();
  configCore->aMag         = ui->core_aMag->text().toFloat();
  configCore->aMagThresh   = ui->core_aMagThresh->text().toFloat();
  configCore->mWeight      = ui->core_mWeight->text().toFloat();
  configCore->mMag         = ui->core_mMag->text().toFloat();
  configCore->mMagThresh   = ui->core_mMagThresh->text().toFloat();
  //configCore->mAng         = ui->mAng->text().toFloat();
  //configCore->mAngThresh   = ui->mAngThresh->text().toFloat();
  //configCore->moveAlpha    = ui->moveAlpha->text().toFloat();
}


/******************************************************************************
* utility function - writes contents of calib structure to GUI 
******************************************************************************/

void windowGUI::calib_write()
{
  ui->rect_gBias0->setText(QString::number(configRect->gBias[0], 'f', 2));
  ui->rect_gBias1->setText(QString::number(configRect->gBias[1], 'f', 2));
  ui->rect_gBias2->setText(QString::number(configRect->gBias[2], 'f', 2));
  ui->rect_gMult0->setText(QString::number(configRect->gMult[0], 'f', 2));
  ui->rect_gMult1->setText(QString::number(configRect->gMult[1], 'f', 2));
  ui->rect_gMult2->setText(QString::number(configRect->gMult[2], 'f', 2));
  ui->rect_gMult3->setText(QString::number(configRect->gMult[3], 'f', 2));
  ui->rect_gMult4->setText(QString::number(configRect->gMult[4], 'f', 2));
  ui->rect_gMult5->setText(QString::number(configRect->gMult[5], 'f', 2));
  ui->rect_gMult6->setText(QString::number(configRect->gMult[6], 'f', 2));
  ui->rect_gMult7->setText(QString::number(configRect->gMult[7], 'f', 2));
  ui->rect_gMult8->setText(QString::number(configRect->gMult[8], 'f', 2));
  ui->rect_aBias0->setText(QString::number(configRect->aBias[0], 'f', 2));
  ui->rect_aBias1->setText(QString::number(configRect->aBias[1], 'f', 2));
  ui->rect_aBias2->setText(QString::number(configRect->aBias[2], 'f', 2));
  ui->rect_aMult0->setText(QString::number(configRect->aMult[0], 'f', 2));
  ui->rect_aMult1->setText(QString::number(configRect->aMult[1], 'f', 2));
  ui->rect_aMult2->setText(QString::number(configRect->aMult[2], 'f', 2));
  ui->rect_aMult3->setText(QString::number(configRect->aMult[3], 'f', 2));
  ui->rect_aMult4->setText(QString::number(configRect->aMult[4], 'f', 2));
  ui->rect_aMult5->setText(QString::number(configRect->aMult[5], 'f', 2));
  ui->rect_aMult6->setText(QString::number(configRect->aMult[6], 'f', 2));
  ui->rect_aMult7->setText(QString::number(configRect->aMult[7], 'f', 2));
  ui->rect_aMult8->setText(QString::number(configRect->aMult[8], 'f', 2));
  ui->rect_mBias0->setText(QString::number(configRect->mBias[0], 'f', 2));
  ui->rect_mBias1->setText(QString::number(configRect->mBias[1], 'f', 2));
  ui->rect_mBias2->setText(QString::number(configRect->mBias[2], 'f', 2));
  ui->rect_mMult0->setText(QString::number(configRect->mMult[0], 'f', 2));
  ui->rect_mMult1->setText(QString::number(configRect->mMult[1], 'f', 2));
  ui->rect_mMult2->setText(QString::number(configRect->mMult[2], 'f', 2));
  ui->rect_mMult3->setText(QString::number(configRect->mMult[3], 'f', 2));
  ui->rect_mMult4->setText(QString::number(configRect->mMult[4], 'f', 2));
  ui->rect_mMult5->setText(QString::number(configRect->mMult[5], 'f', 2));
  ui->rect_mMult6->setText(QString::number(configRect->mMult[6], 'f', 2));
  ui->rect_mMult7->setText(QString::number(configRect->mMult[7], 'f', 2));
  ui->rect_mMult8->setText(QString::number(configRect->mMult[8], 'f', 2));
}


/******************************************************************************
* utility function - read contents of calib structure to GUI 
******************************************************************************/

void windowGUI::calib_read()
{
  configRect->gBias[0] = ui->rect_gBias0->text().toFloat();
  configRect->gBias[1] = ui->rect_gBias1->text().toFloat();
  configRect->gBias[2] = ui->rect_gBias2->text().toFloat();
  configRect->gMult[0] = ui->rect_gMult0->text().toFloat();
  configRect->gMult[1] = ui->rect_gMult1->text().toFloat();
  configRect->gMult[2] = ui->rect_gMult2->text().toFloat();
  configRect->gMult[3] = ui->rect_gMult3->text().toFloat();
  configRect->gMult[4] = ui->rect_gMult4->text().toFloat();
  configRect->gMult[5] = ui->rect_gMult5->text().toFloat();
  configRect->gMult[6] = ui->rect_gMult6->text().toFloat();
  configRect->gMult[7] = ui->rect_gMult7->text().toFloat();
  configRect->gMult[8] = ui->rect_gMult8->text().toFloat();
  configRect->aBias[0] = ui->rect_aBias0->text().toFloat();
  configRect->aBias[1] = ui->rect_aBias1->text().toFloat();
  configRect->aBias[2] = ui->rect_aBias2->text().toFloat();
  configRect->aMult[0] = ui->rect_aMult0->text().toFloat();
  configRect->aMult[1] = ui->rect_aMult1->text().toFloat();
  configRect->aMult[2] = ui->rect_aMult2->text().toFloat();
  configRect->aMult[3] = ui->rect_aMult3->text().toFloat();
  configRect->aMult[4] = ui->rect_aMult4->text().toFloat();
  configRect->aMult[5] = ui->rect_aMult5->text().toFloat();
  configRect->aMult[6] = ui->rect_aMult6->text().toFloat();
  configRect->aMult[7] = ui->rect_aMult7->text().toFloat();
  configRect->aMult[8] = ui->rect_aMult8->text().toFloat();
  configRect->mBias[0] = ui->rect_mBias0->text().toFloat();
  configRect->mBias[1] = ui->rect_mBias1->text().toFloat();
  configRect->mBias[2] = ui->rect_mBias2->text().toFloat();
  configRect->mMult[0] = ui->rect_mMult0->text().toFloat();
  configRect->mMult[1] = ui->rect_mMult1->text().toFloat();
  configRect->mMult[2] = ui->rect_mMult2->text().toFloat();
  configRect->mMult[3] = ui->rect_mMult3->text().toFloat();
  configRect->mMult[4] = ui->rect_mMult4->text().toFloat();
  configRect->mMult[5] = ui->rect_mMult5->text().toFloat();
  configRect->mMult[6] = ui->rect_mMult6->text().toFloat();
  configRect->mMult[7] = ui->rect_mMult7->text().toFloat();
  configRect->mMult[8] = ui->rect_mMult8->text().toFloat();
}


/******************************************************************************
* updates display scale for gyroscope
******************************************************************************/

void windowGUI::glWidget_update()
{
  ui->widget->scaleGyro = ui->disp_scaleGyro->text().toFloat();
  ui->widget->scaleAccl = ui->disp_scaleAccl->text().toFloat();
  ui->widget->scaleMagn = ui->disp_scaleMagn->text().toFloat();
  ui->widget->scaleIMU  = ui->disp_scaleIMU->text().toFloat();
  ui->widget->isGyro    = ui->disp_enableGyro->isChecked();
  ui->widget->isAccl    = ui->disp_enableAccl->isChecked();
  ui->widget->isMagn    = ui->disp_enableMagn->isChecked();
  ui->widget->isIMU     = ui->disp_enableIMU->isChecked();
}


/******************************************************************************
* loads config structure from json file
******************************************************************************/

void windowGUI::on_core_open_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), "../config",
    ("json (*.json)"));
  IMU_engn_load(0, (char *)file.toStdString().c_str(), IMU_engn_core);
  config_write();
}


/******************************************************************************
* saves config structure to json file
******************************************************************************/

void windowGUI::on_core_save_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), "../config",
    ("json (*.json)"));
  IMU_engn_save(0, (char *)file.toStdString().c_str(), IMU_engn_core);
}


/******************************************************************************
* loads calib structure from json file
******************************************************************************/

void windowGUI::on_rect_open_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), "../config", 
    ("json (*.json)"));
  IMU_engn_load(0, (char *)file.toStdString().c_str(), IMU_engn_rect);
  calib_write();
}


/******************************************************************************
* saves calib structure to json file
******************************************************************************/

void windowGUI::on_rect_save_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), "../config",
    ("json (*.json)"));
  IMU_engn_save(0, (char *)file.toStdString().c_str(), IMU_engn_rect);
}


/******************************************************************************
* enables display of gyroscope data
******************************************************************************/

void windowGUI::on_disp_enableGyro_clicked()
{
  ui->disp_enableIMU->setChecked(false);
  glWidget_update();
}


/******************************************************************************
* enables display of accelerometer data
******************************************************************************/

void windowGUI::on_disp_enableAccl_clicked()
{
  ui->disp_enableIMU->setChecked(false);
  glWidget_update();
}


/******************************************************************************
* enables display of accelerometer data
******************************************************************************/

void windowGUI::on_disp_enableMagn_clicked()
{
  ui->disp_enableIMU->setChecked(false);
  glWidget_update();
}


/******************************************************************************
* enables display of IMU results
******************************************************************************/

void windowGUI::on_disp_enableIMU_clicked()
{
  ui->disp_enableGyro->setChecked(false);
  ui->disp_enableAccl->setChecked(false);
  ui->disp_enableMagn->setChecked(false);
  glWidget_update();
}


/******************************************************************************
* change to top-down view
******************************************************************************/

void windowGUI::on_view_up_clicked()
{
  ui->widget->xRot     = 90 * 16;
  ui->widget->yRot     = 0;
  ui->widget->zRot     = 0;
}


/******************************************************************************
* change to side view
******************************************************************************/

void windowGUI::on_view_side1_clicked()
{
  ui->widget->xRot     = 0;
  ui->widget->yRot     = 0;
  ui->widget->zRot     = 0;
}


/******************************************************************************
* change to side view
******************************************************************************/

void windowGUI::on_view_side2_clicked()
{
  ui->widget->xRot     = 0;
  ui->widget->yRot     = 90 * 16;
  ui->widget->zRot     = 0;
}

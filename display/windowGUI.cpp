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
#include <math.h>
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
  IMU_engn_getConfig(0, IMU_engn_pnts, &configUnion);
  configPnts = configUnion.pnts;
  IMU_engn_getConfig(0, IMU_engn_stat, &configUnion);
  configStat = configUnion.stat;
  IMU_engn_getConfig(0, IMU_engn_calb, &configUnion);
  configCalb = configUnion.calb;

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
  // initialize display IF parameters
  ui->disp_scaleGyro->setText(QString::number(config->gyro, 'f', 1));
  ui->disp_scaleAccl->setText(QString::number(config->accl, 'f', 1));
  ui->disp_scaleMagn->setText(QString::number(config->magn, 'f', 1));
  ui->disp_scaleIMU->setText (QString::number(config->imu, 'f', 1));
  glWidget_update();

  // update GUI fields
  core_write();
  rect_write();
  pnts_write();
  stat_write();
  calb_write();
}


/******************************************************************************
* utility function - writes contents of config structure to GUI 
******************************************************************************/

void windowGUI::core_write()
{
  ui->core_isGyro->setChecked(configCore->isGyro);
  ui->core_isAccl->setChecked(configCore->isAccl);
  ui->core_isMagn->setChecked(configCore->isMagn);
  ui->core_isFOM->setChecked(configCore->isFOM);
  ui->core_isTran->setChecked(configCore->isTran);
  ui->core_isPredict->setChecked(configCore->isPredict);
  ui->core_gScale->setText(QString::number(configCore->gScale, 'f', 6));
  ui->core_aWeight->setText(QString::number(configCore->aWeight, 'f', 3));
  ui->core_aMag->setText(QString::number(configCore->aMag, 'f', 2));
  ui->core_aMagThresh->setText(QString::number(configCore->aMagThresh, 'f', 2));
  ui->core_mWeight->setText(QString::number(configCore->mWeight, 'f', 3));
  ui->core_mMag->setText(QString::number(configCore->mMag, 'f', 2));
  ui->core_mMagThresh->setText(QString::number(configCore->mMagThresh, 'f', 2));
  ui->core_mDot->setText(QString::number(configCore->mDot, 'f', 3));
  ui->core_mDotThresh->setText(QString::number(configCore->mDotThresh, 'f', 3));
  ui->core_tranAlpha->setText(QString::number(configCore->tranAlpha, 'f', 2));
}


/******************************************************************************
* utility function - read contents of config structure to GUI 
******************************************************************************/

void windowGUI::core_read()
{
  configCore->isGyro       = ui->core_isGyro->isChecked();
  configCore->isAccl       = ui->core_isAccl->isChecked();
  configCore->isMagn       = ui->core_isMagn->isChecked();
  configCore->isFOM        = ui->core_isFOM->isChecked();
  configCore->isTran       = ui->core_isTran->isChecked();
  configCore->isPredict    = ui->core_isPredict->isChecked();
  configCore->gScale       = ui->core_gScale->text().toFloat();
  configCore->aWeight      = ui->core_aWeight->text().toFloat();
  configCore->aMag         = ui->core_aMag->text().toFloat();
  configCore->aMagThresh   = ui->core_aMagThresh->text().toFloat();
  configCore->mWeight      = ui->core_mWeight->text().toFloat();
  configCore->mMag         = ui->core_mMag->text().toFloat();
  configCore->mMagThresh   = ui->core_mMagThresh->text().toFloat();
  configCore->mDot         = ui->core_mDot->text().toFloat();
  configCore->mDotThresh   = ui->core_mDotThresh->text().toFloat();
  configCore->tranAlpha    = ui->core_tranAlpha->text().toFloat();
}


/******************************************************************************
* utility function - writes contents of calib structure to GUI 
******************************************************************************/

void windowGUI::rect_write()
{
  ui->rect_enable->setChecked(configRect->enable);
  ui->rect_gBias0->setText(QString::number(configRect->gBias[0], 'f', 5));
  ui->rect_gBias1->setText(QString::number(configRect->gBias[1], 'f', 5));
  ui->rect_gBias2->setText(QString::number(configRect->gBias[2], 'f', 5));
  ui->rect_gMult0->setText(QString::number(configRect->gMult[0], 'f', 5));
  ui->rect_gMult1->setText(QString::number(configRect->gMult[1], 'f', 5));
  ui->rect_gMult2->setText(QString::number(configRect->gMult[2], 'f', 5));
  ui->rect_gMult3->setText(QString::number(configRect->gMult[3], 'f', 5));
  ui->rect_gMult4->setText(QString::number(configRect->gMult[4], 'f', 5));
  ui->rect_gMult5->setText(QString::number(configRect->gMult[5], 'f', 5));
  ui->rect_gMult6->setText(QString::number(configRect->gMult[6], 'f', 5));
  ui->rect_gMult7->setText(QString::number(configRect->gMult[7], 'f', 5));
  ui->rect_gMult8->setText(QString::number(configRect->gMult[8], 'f', 5));
  ui->rect_aBias0->setText(QString::number(configRect->aBias[0], 'f', 5));
  ui->rect_aBias1->setText(QString::number(configRect->aBias[1], 'f', 5));
  ui->rect_aBias2->setText(QString::number(configRect->aBias[2], 'f', 5));
  ui->rect_aMult0->setText(QString::number(configRect->aMult[0], 'f', 5));
  ui->rect_aMult1->setText(QString::number(configRect->aMult[1], 'f', 5));
  ui->rect_aMult2->setText(QString::number(configRect->aMult[2], 'f', 5));
  ui->rect_aMult3->setText(QString::number(configRect->aMult[3], 'f', 5));
  ui->rect_aMult4->setText(QString::number(configRect->aMult[4], 'f', 5));
  ui->rect_aMult5->setText(QString::number(configRect->aMult[5], 'f', 5));
  ui->rect_aMult6->setText(QString::number(configRect->aMult[6], 'f', 5));
  ui->rect_aMult7->setText(QString::number(configRect->aMult[7], 'f', 5));
  ui->rect_aMult8->setText(QString::number(configRect->aMult[8], 'f', 5));
  ui->rect_mBias0->setText(QString::number(configRect->mBias[0], 'f', 5));
  ui->rect_mBias1->setText(QString::number(configRect->mBias[1], 'f', 5));
  ui->rect_mBias2->setText(QString::number(configRect->mBias[2], 'f', 5));
  ui->rect_mMult0->setText(QString::number(configRect->mMult[0], 'f', 5));
  ui->rect_mMult1->setText(QString::number(configRect->mMult[1], 'f', 5));
  ui->rect_mMult2->setText(QString::number(configRect->mMult[2], 'f', 5));
  ui->rect_mMult3->setText(QString::number(configRect->mMult[3], 'f', 5));
  ui->rect_mMult4->setText(QString::number(configRect->mMult[4], 'f', 5));
  ui->rect_mMult5->setText(QString::number(configRect->mMult[5], 'f', 5));
  ui->rect_mMult6->setText(QString::number(configRect->mMult[6], 'f', 5));
  ui->rect_mMult7->setText(QString::number(configRect->mMult[7], 'f', 5));
  ui->rect_mMult8->setText(QString::number(configRect->mMult[8], 'f', 5));
}


/******************************************************************************
* utility function - read contents of calib structure to GUI 
******************************************************************************/

void windowGUI::rect_read()
{
  configRect->enable   = ui->rect_enable->isChecked();
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
* utility function - writes contents of config structure to GUI 
******************************************************************************/

void windowGUI::pnts_write()
{
  ui->pnts_enable->setChecked(configPnts->enable);
  ui->pnts_isGyro->setChecked(configPnts->isGyro);
  ui->pnts_isAccl->setChecked(configPnts->isAccl);
  ui->pnts_isMagn->setChecked(configPnts->isMagn);
  ui->pnts_tHold->setText(QString::number(configPnts->tHold/100.0, 'f', 1));
  ui->pnts_tStable->setText(QString::number(configPnts->tStable/100.0, 'f', 1));
  ui->pnts_gAlpha->setText(QString::number(configPnts->gAlpha, 'f', 3));
  ui->pnts_gThresh->setText(QString::number(sqrt(configPnts->gThresh), 'f', 2));
  ui->pnts_aAlpha->setText(QString::number(configPnts->aAlpha, 'f', 3));
  ui->pnts_aThresh->setText(QString::number(sqrt(configPnts->aThresh), 'f', 2));
  ui->pnts_mAlpha->setText(QString::number(configPnts->mAlpha, 'f', 3));
  ui->pnts_mThresh->setText(QString::number(sqrt(configPnts->mThresh), 'f', 2));
}


/******************************************************************************
* utility function - read contents of config structure to GUI 
******************************************************************************/

void windowGUI::pnts_read()
{
  configPnts->enable       = ui->pnts_enable->isChecked();
  configPnts->isGyro       = ui->pnts_isGyro->isChecked();
  configPnts->isAccl       = ui->pnts_isAccl->isChecked();
  configPnts->isMagn       = ui->pnts_isMagn->isChecked();
  configPnts->tHold        = 100.0*ui->pnts_tHold->text().toFloat();
  configPnts->tStable      = 100.0*ui->pnts_tStable->text().toFloat();
  configPnts->gAlpha       = ui->pnts_gAlpha->text().toFloat();
  configPnts->gThresh      = pow(ui->pnts_gThresh->text().toFloat(),2);
  configPnts->aAlpha       = ui->pnts_aAlpha->text().toFloat();
  configPnts->aThresh      = pow(ui->pnts_aThresh->text().toFloat(),2);
  configPnts->mAlpha       = ui->pnts_mAlpha->text().toFloat();
  configPnts->mThresh      = pow(ui->pnts_mThresh->text().toFloat(),2);
}


/******************************************************************************
* utility function - writes contents of config structure to GUI 
******************************************************************************/

void windowGUI::stat_write()
{
  ui->stat_enable->setChecked(configStat->enable);
  ui->stat_alpha->setText(QString::number(configStat->alpha, 'f', 5));
}


/******************************************************************************
* utility function - read contents of config structure to GUI 
******************************************************************************/

void windowGUI::stat_read()
{
  configStat->enable       = ui->stat_enable->isChecked();
  configStat->alpha        = ui->stat_alpha->text().toFloat();
}


/******************************************************************************
* utility function - writes contents of config structure to GUI 
******************************************************************************/

void windowGUI::calb_write()
{
  ui->calb_enable->setChecked(configCalb->enable);
}


/******************************************************************************
* utility function - read contents of config structure to GUI 
******************************************************************************/

void windowGUI::calb_read()
{
  configCalb->enable       = ui->calb_enable->isChecked();
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
  core_write();
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
  rect_write();
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
* loads config structure from json file
******************************************************************************/

void windowGUI::on_pnts_open_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), "../config",
    ("json (*.json)"));
  IMU_engn_load(0, (char *)file.toStdString().c_str(), IMU_engn_pnts);
  pnts_write();
}


/******************************************************************************
* saves config structure to json file
******************************************************************************/

void windowGUI::on_pnts_save_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), "../config",
    ("json (*.json)"));
  IMU_engn_save(0, (char *)file.toStdString().c_str(), IMU_engn_pnts);
}


/******************************************************************************
* loads config structure from json file
******************************************************************************/

void windowGUI::on_stat_open_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), "../config",
    ("json (*.json)"));
  IMU_engn_load(0, (char *)file.toStdString().c_str(), IMU_engn_stat);
  stat_write();
}


/******************************************************************************
* saves config structure to json file
******************************************************************************/

void windowGUI::on_stat_save_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), "../config",
    ("json (*.json)"));
  IMU_engn_save(0, (char *)file.toStdString().c_str(), IMU_engn_stat);
}


/******************************************************************************
* loads config structure from json file
******************************************************************************/

void windowGUI::on_calb_open_clicked()
{
  QString file = QFileDialog::getOpenFileName(this, ("Open File"), "../config",
    ("json (*.json)"));
  IMU_engn_load(0, (char *)file.toStdString().c_str(), IMU_engn_calb);
  calb_write();
}


/******************************************************************************
* saves config structure to json file
******************************************************************************/

void windowGUI::on_calb_save_clicked()
{
  QString file = QFileDialog::getSaveFileName(this, ("Save File"), "../config",
    ("json (*.json)"));
  IMU_engn_save(0, (char *)file.toStdString().c_str(), IMU_engn_calb);
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

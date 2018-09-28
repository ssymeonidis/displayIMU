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

#ifndef WINDOWGUI_H
#define WINDOWGUI_H

// include statements
#include <QMainWindow>
#include "IMU.h"

// create abreviated namespace for windowGUI (qtcreator codegen)
namespace Ui {class windowGUI;}

// define the overarching class (started with qtcreator codegen)
class windowGUI : public QMainWindow {
  Q_OBJECT

public:
  explicit windowGUI(QWidget *parent = 0);
  ~windowGUI();
  void initIMU(char* config_file, char* calib_file);

private slots:
  void config_read();
  void config_write();
  void calib_read();
  void calib_write(); 
  void glWidget_update();
  void on_configOpen_clicked();
  void on_configSave_clicked();
  void on_calibOpen_clicked();
  void on_calibSave_clicked();
  void on_dispEnableGyro_clicked();
  void on_dispEnableAccl_clicked();
  void on_dispEnableMagn_clicked();
  void on_dispEnableIMU_clicked();
  void on_viewUp_clicked();
  void on_viewSide1_clicked();
  void on_viewSide2_clicked();

private:
  // internal structures/classes
  Ui::windowGUI      *ui;
  displayIMU_config  *config;
  displayIMU_calib   *calib;

  // internal functions
  void load_json(char* filename);
};

#endif // WINDOWGUI_H

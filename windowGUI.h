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
  windowGUI(char* config_file, char* calib_file);
  ~windowGUI();

private:
  // internal structures/classes
  Ui::windowGUI      *ui;
  displayIMU_config  *config;
  displayIMU_calib   *calib;

  //internal functions
  void config_read(void);
  void config_write(void);
  void calib_read(void);
  void calib_write(void); 
};

#endif // WINDOWGUI_H

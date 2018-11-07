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

#ifndef WINDOWGUI_H
#define WINDOWGUI_H

// include statements
#include <QMainWindow>
#include "IMU_engn.h"
#include "configGUI.h"

// create abreviated namespace for windowGUI (qtcreator codegen)
namespace Ui {class windowGUI;}

// define the overarching class (started with qtcreator codegen)
class windowGUI : public QMainWindow {
  Q_OBJECT

public:
  explicit windowGUI(QWidget *parent = 0);
  ~windowGUI();
  void initIMU(configGUI *config);

private slots:
  void glWidget_update();
  void core_read();
  void core_write();
  void rect_read();
  void rect_write();
  void pnts_read();
  void pnts_write();
  void stat_read();
  void stat_write();
  void calb_read();
  void calb_write(); 
  void on_core_open_clicked();
  void on_core_save_clicked();
  void on_rect_open_clicked();
  void on_rect_save_clicked();
  void on_pnts_open_clicked();
  void on_pnts_save_clicked();
  void on_stat_open_clicked();
  void on_stat_save_clicked();
  void on_disp_enableGyro_clicked();
  void on_disp_enableAccl_clicked();
  void on_disp_enableMagn_clicked();
  void on_disp_enableIMU_clicked();
  void on_view_up_clicked();
  void on_view_side1_clicked();
  void on_view_side2_clicked();

private:
  // internal structures/classes
  IMU_core_config     *configCore;
  IMU_rect_config     *configRect;
  IMU_pnts_config     *configPnts;
  IMU_stat_config     *configStat;
  IMU_calb_config     *configCalb;
  Ui::windowGUI       *ui;
};

#endif // WINDOWGUI_H

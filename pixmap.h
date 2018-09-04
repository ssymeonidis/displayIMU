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

#ifndef PIXMAP_H
#define PIXMAP_H

#include <string>
#include <QPixmap>
#include <QWidget>
#include <QLabel>
#include <QBoxLayout>
#include <QLineEdit>
#include <QSlider>
#include <QPushButton>
#include <QTimer>

class pixmap : public QWidget
{
  Q_OBJECT

  public:
    pixmap(const char* video_path, int offset, float rate, int max_frame);

  public slots:
    void               paintEvent();
    void               updateParams();
    void               updateIndex1(int value);
    void               updateIndex2();
    void               updatePlay();

  private:
    // QT user interface objects
    QTimer*            refresh_timer;
    QPixmap            pixels; 
    QLabel*            label;
    QLabel*            offsetText;
    QLineEdit*         offsetEdit;
    QLabel*            rateText;
    QLineEdit*         rateEdit;
    QPushButton*       timePlay;
    QLabel*            timeText;
    QSlider*           timeSlider;
    QLineEdit*         timeEdit;

    // internal variables
    bool               is_play;
    int                offset;
    float              rate;
    char*              video_path;
    char               file_name[100];
    char               file_ext[5];
};

#endif

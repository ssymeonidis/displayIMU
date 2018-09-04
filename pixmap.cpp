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

#include "pixmap.h"
#include "dataread.h"
#include <stdio.h>


pixmap::pixmap(const char* video_path_in, int offset_in, float rate_in, int max_frame)
{
  // save off input params
  is_play            = true;
  offset             = offset_in;
  rate               = rate_in;
  int str_size       = strlen(video_path_in) + 1;
  video_path         = (char*) malloc (str_size * sizeof(char));
  strcpy(video_path, video_path_in); 
  strcpy(file_ext, ".jpg");

  // create all the children objects
  offsetText           = new QLabel();
  offsetText->setText("offset");
  offsetEdit           = new QLineEdit(QString::number(offset));
  rateText             = new QLabel();
  rateText->setText("rate");
  rateEdit             = new QLineEdit(QString::number(rate));
  timeText             = new QLabel();
  timeText->setText("time slider (warning: this control will corrupt IMU csv output)");
  timePlay             = new QPushButton(">");
  timePlay->setFixedSize(QSize(29,29));
  timeSlider           = new QSlider(Qt::Horizontal);
  timeSlider->setRange(1,max_frame);
  timeEdit             = new QLineEdit(QString::number(1));
  timeEdit->setFixedSize(QSize(90,29));
  QHBoxLayout *params  = new QHBoxLayout();
  params->addWidget(offsetText);
  params->addWidget(offsetEdit);
  params->addWidget(rateText);
  params->addWidget(rateEdit);
  QHBoxLayout *time    = new QHBoxLayout();
  time->addWidget(timePlay);
  time->addWidget(timeSlider);
  time->addWidget(timeEdit);
  QVBoxLayout *ctrls   = new QVBoxLayout();
  ctrls->addSpacing(150);
  ctrls->addLayout(params);
  ctrls->addWidget(timeText);
  ctrls->addLayout(time);
  ctrls->addSpacing(150);
  label                = new QLabel();
  QHBoxLayout *main    = new QHBoxLayout(this);
  main->setMargin(0);
  main->addWidget(label);
  main->addLayout(ctrls);

  // connect widgets to slot
  connect(offsetEdit,  SIGNAL(editingFinished()), this, SLOT(updateParams()));
  connect(rateEdit,    SIGNAL(editingFinished()), this, SLOT(updateParams()));
  connect(timePlay,    SIGNAL(released()),        this, SLOT(updatePlay()));
  connect(timeSlider,  SIGNAL(valueChanged(int)), this, SLOT(updateIndex1(int)));
  connect(timeEdit,    SIGNAL(editingFinished()), this, SLOT(updateIndex2()));

  // create timer
  refresh_timer = new QTimer(this);
  connect(refresh_timer, SIGNAL(timeout()), this, SLOT(paintEvent()));
  refresh_timer->start(10);

  // debug message
  printf("creating video from %s...\n", video_path); 
}


void pixmap::paintEvent()
{
  int index = (csv_buffer_index+offset)*rate;
  sprintf(file_name, "%s%06d%s", video_path, index, file_ext);
  pixels.load(file_name);
  label->setPixmap(pixels);
  if (is_play == true) {
    QString text = QString::number(csv_buffer_index);
    timeEdit->setText(text);
    timeSlider->setValue(csv_buffer_index);  
  }
}


void pixmap::updateParams()
{
  QString textOffset    = offsetEdit->text();
  offset                = textOffset.toInt();
  QString textRate      = rateEdit->text();
  rate                  = textRate.toFloat(); 
}


void pixmap::updateIndex1(int value)
{
  if (is_play == false) {
    csv_buffer_index    = value;
  }
} 


void pixmap::updateIndex2()
{
  if (is_play == false) {
    QString textIndex   = timeEdit->text();
    csv_buffer_index    = textIndex.toInt();
  }
}


void pixmap::updatePlay()
{
  is_play               = !is_play;
}  

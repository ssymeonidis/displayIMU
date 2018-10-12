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

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <GL/glu.h>
#include "IMU_engn.h"


class GLWidget : public QGLWidget
{
  Q_OBJECT

  public:
    explicit GLWidget(QWidget *parent = 0);
    ~GLWidget();

    // draw parameters
    int    xRot;
    int    yRot;
    int    zRot;
    bool   isGyro;
    bool   isAccl;
    bool   isMagn;
    bool   isIMU;
    float  scaleGyro;
    float  scaleAccl;
    float  scaleMagn;
    float  scaleIMU;

  public slots:
    void   updateFrame();

  protected:
    void   initializeGL();
    void   paintGL();
    void   resizeGL(int width, int height);
    void   mousePressEvent(QMouseEvent *event);
    void   mouseMoveEvent(QMouseEvent *event);

  private:
    // internal functions
    void   drawArrow(GLfloat faceColor[4], GLfloat scale, GLfloat angles[2]);
    void   drawVector(GLfloat faceColor[4], GLfloat vector[3], GLfloat scale);
    void   drawGrid();

    // internal objects/parameters
    IMU_engn_sensor  *sensor;
    GLUquadricObj    *obj;
    QTimer           *refresh_timer;
    QPoint           lastPos;
};

#endif

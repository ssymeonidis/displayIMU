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

#include <QtGui>
#include <QtOpenGL>
#include <QTimer>
#include <math.h>
#include "glwidget.h"
#include "window.h"
#include "receive.h"
#include "dataread.h"
#include "MARG.h"

GLfloat GLWidget::acclColor[4] = {   1.0,    0.0,    0.0,    1.0};
GLfloat GLWidget::magColor[4]  = {   0.0,    1.0,    0.0,    1.0};
GLfloat GLWidget::gyroColor[4] = {   0.0,    0.0,    1.0,    1.0};
GLfloat GLWidget::gridColor[4] = {   0.3,    0.3,    0.3,    1.0}; 

GLWidget::GLWidget(int is_csv_file_in)
  : QGLWidget(QGLFormat(QGL::SampleBuffers), 0)
{
  // capture csv state
  is_csv_file  = is_csv_file_in;

  // initialize internal display variables
  xRot       = 0;
  yRot       = 0;
  zRot       = 0;

  // initialize component state variables
  isAccl     = true;
  isMag      = true;
  isGyro     = false;
  isIMU      = false;
  scaleAccl  = 2500;
  scaleMag   = 5000;
  scaleGyro  = 200; 

  // create timer
  refresh_timer = new QTimer(this);
  connect(refresh_timer, SIGNAL(timeout()), this, SLOT(updateFrame()));
  refresh_timer->start(10);
}


GLWidget::~GLWidget()
{
}


QSize GLWidget::minimumSizeHint() const
{
  return QSize(50, 50);
}


QSize GLWidget::sizeHint() const
{
  return QSize(400, 400);
}


static void qNormalizeAngle(int &angle)
{
  while (angle < 0)
    angle += 360 * 16;
  while (angle > 360 * 16)
    angle -= 360 * 16;
}


void GLWidget::setXRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != xRot) {
    xRot = angle;
    emit xRotationChanged(angle);
    updateGL();
  }
}


void GLWidget::setYRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != yRot) {
    yRot = angle;
    emit yRotationChanged(angle);
    updateGL();
  }
}


void GLWidget::setZRotation(int angle)
{
  qNormalizeAngle(angle);
  if (angle != zRot) {
    zRot = angle;
    emit zRotationChanged(angle);
    updateGL();
  }
}


void GLWidget::updateFrame()
{
  updateGL();
}


void GLWidget::initializeGL()
{
  // create glu object
  obj           = gluNewQuadric();

  // configure OpenGL
  glClearColor(0.0, 0.0, 0.0, 1.0);
  //glEnable(GL_DEPTH_TEST);
  //glEnable(GL_CULL_FACE);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  static GLfloat lightPosition1[4] = {  0.5,  5.0,  7.0, 1.0 };
  static GLfloat lightDiffuse1[4]  = {  0.5,  0.5,  0.5, 1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition1);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  lightDiffuse1);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  lightDiffuse1);
  glEnable(GL_BLEND); 
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
  //glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}


void GLWidget::paintGL()
{
  // initializing the variables
  int i;
  if (is_csv_file == 0)
    i = sensor_buffer_index;
  else
    i = csv_buffer_index;

  // setup camera
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -10.0);
  glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
  glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
  glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

  // draw reference grid
  drawGrid(); 

  // draw accelerometer state
  if (isAccl == true) {
    if (is_csv_file == 0)
      drawVector(acclColor, &(sensor_buffer[i][3]), scaleAccl);
    else
      drawVector(acclColor, &(csv_buffer[i][4]), scaleAccl);
  }

  // draw magnetometer state
  if (isMag == true) {
    if (is_csv_file == 0) 
      drawVector(magColor, &(sensor_buffer[i][6]), scaleMag);
    else
      drawVector(magColor, &(csv_buffer[i][1]), scaleMag);
  }

  // draw gyroscope state
  if (isGyro == true) {
    if (is_csv_file == 0) 
      drawVector(gyroColor, &(sensor_buffer[i][0]), scaleGyro);
    else
      drawVector(gyroColor, &(csv_buffer[i][7]), scaleGyro);
  }

  // draw imu state
  if (isIMU == true) {
    glPushMatrix();
      if (is_csv_file == 0) {
        glRotatef(sensor_buffer[i][11], 1.0, 0.0, 0.0);
        glRotatef(sensor_buffer[i][10], 0.0, 0.0, 1.0);
        glRotatef(sensor_buffer[i][9],  0.0, 1.0, 0.0);
        glTranslatef( sensor_buffer[i][12]/scaleIMU,
                      sensor_buffer[i][14]/scaleIMU,
                      sensor_buffer[i][13]/scaleIMU);
      } else {
        glRotatef(csv_buffer[i][12], 1.0, 0.0, 0.0);
        glRotatef(csv_buffer[i][11], 0.0, 0.0, 1.0);
        glRotatef(csv_buffer[i][10], 0.0, 1.0, 0.0);
        glTranslatef( csv_buffer[i][13]/scaleIMU,
                      csv_buffer[i][15]/scaleIMU,
                      csv_buffer[i][14]/scaleIMU);
      }
      GLfloat vector1[3]    = { 0.0,  1.0,  0.0};
      drawVector(gyroColor, vector1, 1.0);
      GLfloat vector2[3]    = {-0.0,  0.0,  1.0};
      drawVector(acclColor, vector2, 1.0);
      GLfloat vector3[3]    = {-1.0,  0.0,  0.0};
      drawVector(magColor,  vector3, 1.0);
    glPopMatrix();
  }

  // update debug params
  /*deltaGrav->setText(QString::number(delta_G));
  deltaNorm->setText(QString::number(delta_M));
  deltaAng->setText(QString::number(delta_ang));
  deltaAccl->setText(QString::number(delta_a));
  deltaMag->setText(QString::number(delta_m));*/
}


void GLWidget::resizeGL(int width, int height)
{
  int side = qMin(width, height);
  glViewport((width - side) / 2, (height - side) / 2, side, side);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-1.0, +1.0, -1.0, +1.0, 4.0, 15.0);
  glMatrixMode(GL_MODELVIEW);
}


void GLWidget::mousePressEvent(QMouseEvent *event)
{
  lastPos = event->pos();
}


void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
  int dx = event->x() - lastPos.x();
  int dy = event->y() - lastPos.y();

  if (event->buttons() & Qt::LeftButton) {
    setXRotation(xRot + 8 * dy);
    setYRotation(yRot + 8 * dx);
  } else if (event->buttons() & Qt::RightButton) {
    setXRotation(xRot + 8 * dy);
    setZRotation(zRot + 8 * dx);
  }
  lastPos = event->pos();
}


void GLWidget::drawArrow(GLfloat faceColor[4], GLfloat scale, GLfloat angles[2])
{
  float cylinderHeight = arrowSize - coneHeight;
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, faceColor);
  glPushMatrix();
    glRotatef(angles[1], 0.0, 1.0, 0.0);
    glRotatef(angles[0], 1.0, 0.0, 0.0);
    glScalef(1.0, 1.0, scale);
    gluCylinder(obj,cylinderRadius,cylinderRadius,cylinderHeight,10,1);
    glPushMatrix();
      glRotatef(180.0, 0.0, 1.0, 0.0);
      gluDisk(obj, 0, cylinderRadius, 10, 1);
      glRotatef(180.0, 0.0, 1.0, 0.0);
      glTranslatef(0, 0, cylinderHeight);
      gluCylinder(obj, coneRadius, 0, coneHeight, 10, 1);
      glRotatef(180.0, 0.0, 1.0, 0.0);
      gluDisk(obj, cylinderRadius, coneRadius, 10, 1);
    glPopMatrix();
  glPopMatrix();
}


void GLWidget::drawVector(GLfloat faceColor[4], GLfloat vector[3], GLfloat scale)
{
  // extracting the components for readability
  float x =  vector[1]/scale;
  float y = -vector[0]/scale;
  float z = -vector[2]/scale;
  
  // convert to sperical cordinates
  float r = sqrt(x*x + y*y + z*z);
  float t = 180*asin(z/r)/M_PI;
  float p = 180*atan2(y,x)/M_PI;
  
  // perform the draw
  float angle[2] = {t,p};
  drawArrow(faceColor, r, angle);
  glBegin(GL_LINES);
  glVertex3f( 0.0, 0.0,  0.0); 
  glVertex3f( y/2, 0.0,  x/2);
  glEnd();
}


void GLWidget::drawGrid()
{
  // draw the reference plane
  GLfloat planeColor1[4] = {1.0, 0.0, 1.0, 1.0};
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, planeColor1);
  glBegin(GL_LINES);
    glVertex3f(-1.0, 0.0,  1.0); glVertex3f(-1.0, 0.0, -1.0);
    glVertex3f(-0.8, 0.0,  1.0); glVertex3f(-0.8, 0.0, -1.0);
    glVertex3f(-0.6, 0.0,  1.0); glVertex3f(-0.6, 0.0, -1.0);
    glVertex3f(-0.4, 0.0,  1.0); glVertex3f(-0.4, 0.0, -1.0);
    glVertex3f(-0.2, 0.0,  1.0); glVertex3f(-0.2, 0.0, -1.0);
    glVertex3f( 0.0, 0.0,  1.0); glVertex3f(-0.0, 0.0, -1.0);
    glVertex3f( 0.2, 0.0,  1.0); glVertex3f( 0.2, 0.0, -1.0);
    glVertex3f( 0.4, 0.0,  1.0); glVertex3f( 0.4, 0.0, -1.0);
    glVertex3f( 0.6, 0.0,  1.0); glVertex3f( 0.6, 0.0, -1.0);
    glVertex3f( 0.8, 0.0,  1.0); glVertex3f( 0.8, 0.0, -1.0);
    glVertex3f( 1.0, 0.0,  1.0); glVertex3f( 1.0, 0.0, -1.0);
    glVertex3f( 1.0, 0.0, -1.0); glVertex3f(-1.0, 0.0, -1.0);
    glVertex3f( 1.0, 0.0, -0.8); glVertex3f(-1.0, 0.0, -0.8);
    glVertex3f( 1.0, 0.0, -0.6); glVertex3f(-1.0, 0.0, -0.6);
    glVertex3f( 1.0, 0.0, -0.4); glVertex3f(-1.0, 0.0, -0.4);
    glVertex3f( 1.0, 0.0, -0.2); glVertex3f(-1.0, 0.0, -0.2);
    glVertex3f( 1.0, 0.0,  0.0); glVertex3f(-1.0, 0.0,  0.0);
    glVertex3f( 1.0, 0.0,  0.2); glVertex3f(-1.0, 0.0,  0.2);
    glVertex3f( 1.0, 0.0,  0.4); glVertex3f(-1.0, 0.0,  0.4);
    glVertex3f( 1.0, 0.0,  0.6); glVertex3f(-1.0, 0.0,  0.6);
    glVertex3f( 1.0, 0.0,  0.8); glVertex3f(-1.0, 0.0,  0.8);
    glVertex3f( 1.0, 0.0,  1.0); glVertex3f(-1.0, 0.0,  1.0);
  glEnd();
}

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
#include <QtGui>
#include <QtOpenGL>
#include <QTimer>
#include <math.h>
#include "windowGUI.h"
#include "glWidget.h"

// internal constants
static const float arrowSize      = 0.5;
static const float coneHeight     = 0.05;
static const float coneRadius     = 0.03;
static const float cylinderRadius = 0.012;

// colors for GUI elements
static GLfloat acclColor[4] = {1.0, 0.0, 0.0, 1.0};
static GLfloat magnColor[4] = {0.0, 1.0, 0.0, 1.0};
static GLfloat gyroColor[4] = {0.0, 0.0, 1.0, 1.0};
static GLfloat gridColor[4] = {1.0, 0.0, 1.0, 1.0}; 


/******************************************************************************
* constructor for the OpenGL sensor/IMU display widget
******************************************************************************/

GLWidget::GLWidget(QWidget *parent) : 
  QGLWidget(QGLFormat(QGL::SampleBuffers), 
  parent)
{
  // initialize internal display variables
  xRot       = 120;
  yRot       = 700;
  zRot       = 0;

  // initialize component state variables
  isAccl     = true;
  isMagn     = true;
  isGyro     = true;
  isIMU      = false;
  scaleAccl  = 1;
  scaleMagn  = 1;
  scaleGyro  = 1; 
  
  // get pointer to the sensor data
  IMU_union_config configIMU;
  IMU_engn_getConfig(0, IMU_engn_self, &configIMU);
  configIMU.engn->isSensorStruct = 1;
  IMU_engn_getSensor(0, &sensor);

  // create timer
  refresh_timer = new QTimer(this);
  connect(refresh_timer, SIGNAL(timeout()), this, SLOT(updateFrame()));
  refresh_timer->start(10);
}


/******************************************************************************
* placeholder for deconstructor
******************************************************************************/

GLWidget::~GLWidget()
{
}


/******************************************************************************
* function used by timer to update display
******************************************************************************/

void GLWidget::updateFrame()
{
  updateGL();
}


/******************************************************************************
* function used to initialize the real-time display
******************************************************************************/

void GLWidget::initializeGL()
{
  // create glu object
  obj           = gluNewQuadric();

  // configure OpenGL
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glShadeModel(GL_SMOOTH);
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  static GLfloat lightPosition[4] = {  0.5,  5.0,  7.0, 1.0 };
  static GLfloat lightDiffuse[4]  = {  0.5,  0.5,  0.5, 1.0 };
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  lightDiffuse);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  lightDiffuse);
  glEnable(GL_BLEND); 
  glBlendFunc(GL_SRC_ALPHA, GL_ONE);
}


/******************************************************************************
* function used to update the real-time display
******************************************************************************/

void GLWidget::paintGL()
{
  // setup camera
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  glTranslatef(0.0, 0.0, -10.0);
  glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
  glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
  glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

  // draw reference grid
  drawGrid(); 

  // draw gyroscope state
  if (isGyro == true){
    float val[3] = {(float)sensor->gRaw[0], (float)sensor->gRaw[1],
                    (float)sensor->gRaw[2]};
    drawVector(gyroColor, val, scaleGyro);
  }

  // draw accelerometer state
  if (isAccl == true) {
    float val[3] = {(float)sensor->aRaw[0], (float)sensor->aRaw[1],
                    (float)sensor->aRaw[2]};
    drawVector(acclColor, val, scaleAccl);
  }

  // draw magnetometer state
  if (isMagn == true) {
    float val[3] = {(float)sensor->mRaw[0], (float)sensor->mRaw[1],
                    (float)sensor->mRaw[2]};
    drawVector(magnColor, val, scaleMagn);
  }

  // draw imu state
  if (isIMU == true) {
    IMU_engn_estm estm;
    IMU_engn_getEstm(0, 0, &estm);
    glPushMatrix();
    glRotatef(estm.ang[2], 1.0, 0.0, 0.0);
    glRotatef(estm.ang[1], 0.0, 0.0, 1.0);
    glRotatef(estm.ang[0], 0.0, 1.0, 0.0);
    glTranslatef((float)estm.tran[0]/scaleIMU,
                 (float)estm.tran[2]/scaleIMU,
                 (float)estm.tran[1]/scaleIMU);
    GLfloat vector1[3]    = { 0.0,  1.0,  0.0};
    drawVector(gyroColor, vector1, 1.0);
    GLfloat vector2[3]    = {-0.0,  0.0,  1.0};
    drawVector(acclColor, vector2, 1.0);
    GLfloat vector3[3]    = {-1.0,  0.0,  0.0};
    drawVector(magnColor, vector3, 1.0);
    glPopMatrix();
  }
}


/******************************************************************************
* function used to resize the real-time display
******************************************************************************/

void GLWidget::resizeGL(int width, int height)
{
  int side = qMin(width, height);
  glViewport((width - side) / 2, (height - side) / 2, side, side);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-1.0, +1.0, -1.0, +1.0, 4.0, 15.0);
  glMatrixMode(GL_MODELVIEW);
}


/******************************************************************************
* function used to capture mouse position for display rotation
******************************************************************************/

void GLWidget::mousePressEvent(QMouseEvent *event)
{
  lastPos = event->pos();
}


/******************************************************************************
* function used to rotate display when draging mouse across OpenGL window
******************************************************************************/

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
  int dx = event->x() - lastPos.x();
  int dy = event->y() - lastPos.y();

  if (event->buttons() & Qt::LeftButton) {
    xRot = xRot + 8 * dy;
    yRot = yRot + 8 * dx;
  } 
  lastPos = event->pos();
}


/******************************************************************************
* function used to draw an arrow (cylinders and disks make for more visible and
* more plesant arrows than line draws)
******************************************************************************/

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


/******************************************************************************
* function used to draw a vector (uses draw angle as basis function)
******************************************************************************/

void GLWidget::drawVector(GLfloat faceColor[4], float vector[3], GLfloat scale)
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


/******************************************************************************
* function used to grid (helps visualization of arrows under differen rotations
******************************************************************************/

void GLWidget::drawGrid()
{
  // draw the reference plane
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, gridColor);
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

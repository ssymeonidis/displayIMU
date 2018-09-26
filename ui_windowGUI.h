/********************************************************************************
** Form generated from reading UI file 'windowGUI.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WINDOWGUI_H
#define UI_WINDOWGUI_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "glWidget.h"

QT_BEGIN_NAMESPACE

class Ui_windowGUI
{
public:
    QWidget *centralWidget;
    QLineEdit *dispScaleGyro;
    QLineEdit *dispScaleAccl;
    QLineEdit *dispScaleMagn;
    QLineEdit *dispScaleIMU;
    QCheckBox *dispEnableGyro;
    QCheckBox *dispEnableAccl;
    QFrame *line_v1;
    QCheckBox *dispEnableMagn;
    QCheckBox *dispEnableIMU;
    QLabel *labelDisplay;
    QFrame *line_h1;
    QLabel *labelControls;
    QCheckBox *noGyro;
    QCheckBox *noAccl;
    QCheckBox *noMagn;
    QPushButton *viewUp;
    QPushButton *viewSide1;
    QPushButton *viewSide2;
    QLineEdit *up0;
    QLabel *labelUp;
    QLineEdit *up1;
    QLineEdit *up2;
    QPushButton *selectReference;
    QPushButton *selectReset;
    QPushButton *selectDeadRecon;
    QLineEdit *gBias0;
    QLineEdit *gBias1;
    QLineEdit *gBias2;
    QLineEdit *gMult1;
    QLineEdit *gMult2;
    QLineEdit *gMult0;
    QLineEdit *gMult5;
    QLineEdit *gMult3;
    QLineEdit *gMult4;
    QLineEdit *gMult8;
    QLineEdit *gMult6;
    QLineEdit *gMult7;
    QLabel *labelCalib1;
    QLabel *labelCalib2;
    QLineEdit *aBias1;
    QLineEdit *aMult2;
    QLineEdit *aMult7;
    QLineEdit *aMult1;
    QLineEdit *aMult0;
    QLineEdit *aMult3;
    QLabel *labelCalib4;
    QLineEdit *aMult8;
    QLineEdit *aBias2;
    QLineEdit *aBias0;
    QLineEdit *aMult5;
    QLineEdit *aMult6;
    QLabel *labelCalib3;
    QLineEdit *aMult4;
    QLineEdit *mBias1;
    QLineEdit *mMult3;
    QLineEdit *mMult2;
    QLineEdit *mMult6;
    QLineEdit *mMult1;
    QLabel *labelCalib6;
    QLineEdit *mBias0;
    QLineEdit *mMult0;
    QLineEdit *mBias2;
    QLineEdit *mMult4;
    QLabel *labelCalib5;
    QLineEdit *mMult8;
    QLineEdit *mMult5;
    QLineEdit *mMult7;
    QFrame *line_h2;
    QLineEdit *gMag;
    QLineEdit *aMag;
    QLabel *labelCalib7;
    QLineEdit *gAng;
    QLabel *labelCalib8;
    QLabel *labelCalib9;
    QLabel *labelConfig1;
    QLabel *labelConfig;
    QFrame *line_v2;
    QLabel *labelGyroBias;
    QLabel *labelGyroScale;
    QLabel *labelAcclScale;
    QLabel *labelAcclBias;
    QLabel *labelMagnBias;
    QLabel *labelAcclMag;
    QLabel *labelMagnMag;
    QLabel *labelMagnAng;
    QLabel *textMagnBias;
    QLabel *textAcclMag;
    QLabel *textGyroScale;
    QLabel *textAcclScale;
    QLabel *textAcclBias;
    QLabel *textMagnMag;
    QLabel *textMagnAng;
    QLabel *textGyroBias;
    GLWidget *widget;
    QLabel *labelConfig_2;
    QPushButton *calibOpen;
    QPushButton *calibSave;
    QCheckBox *noWeight;
    QCheckBox *noTear;
    QCheckBox *noMove;
    QCheckBox *noAutocal;
    QCheckBox *noFOM;
    QCheckBox *noFltr;
    QLineEdit *gThreshVal;
    QLineEdit *gThreshTime;
    QLabel *labelConfig2;
    QLabel *labelConfig4;
    QLabel *labelConfig3;
    QLineEdit *aWeight;
    QLineEdit *aAlpha;
    QLineEdit *mAlpha;
    QLabel *labelConfig6;
    QLineEdit *mWeight;
    QLabel *labelConfig5;
    QLineEdit *autocalAlpha;
    QLabel *labelConfig8;
    QPushButton *configSave;
    QPushButton *configOpen;
    QRadioButton *dispVectRaw;
    QRadioButton *dispVectCor;
    QRadioButton *dispVectFltr;
    QLabel *labelConfig7;
    QLineEdit *moveAlpha;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *windowGUI)
    {
        if (windowGUI->objectName().isEmpty())
            windowGUI->setObjectName(QStringLiteral("windowGUI"));
        windowGUI->resize(1400, 770);
        QFont font;
        font.setBold(false);
        font.setWeight(50);
        windowGUI->setFont(font);
        centralWidget = new QWidget(windowGUI);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        dispScaleGyro = new QLineEdit(centralWidget);
        dispScaleGyro->setObjectName(QStringLiteral("dispScaleGyro"));
        dispScaleGyro->setGeometry(QRect(430, 40, 101, 27));
        dispScaleAccl = new QLineEdit(centralWidget);
        dispScaleAccl->setObjectName(QStringLiteral("dispScaleAccl"));
        dispScaleAccl->setGeometry(QRect(430, 80, 101, 27));
        dispScaleMagn = new QLineEdit(centralWidget);
        dispScaleMagn->setObjectName(QStringLiteral("dispScaleMagn"));
        dispScaleMagn->setGeometry(QRect(430, 120, 101, 27));
        dispScaleIMU = new QLineEdit(centralWidget);
        dispScaleIMU->setObjectName(QStringLiteral("dispScaleIMU"));
        dispScaleIMU->setGeometry(QRect(430, 160, 101, 27));
        dispEnableGyro = new QCheckBox(centralWidget);
        dispEnableGyro->setObjectName(QStringLiteral("dispEnableGyro"));
        dispEnableGyro->setGeometry(QRect(540, 40, 141, 22));
        dispEnableAccl = new QCheckBox(centralWidget);
        dispEnableAccl->setObjectName(QStringLiteral("dispEnableAccl"));
        dispEnableAccl->setGeometry(QRect(540, 80, 141, 22));
        line_v1 = new QFrame(centralWidget);
        line_v1->setObjectName(QStringLiteral("line_v1"));
        line_v1->setGeometry(QRect(710, 0, 20, 701));
        line_v1->setFrameShape(QFrame::VLine);
        line_v1->setFrameShadow(QFrame::Sunken);
        dispEnableMagn = new QCheckBox(centralWidget);
        dispEnableMagn->setObjectName(QStringLiteral("dispEnableMagn"));
        dispEnableMagn->setGeometry(QRect(540, 120, 141, 22));
        dispEnableIMU = new QCheckBox(centralWidget);
        dispEnableIMU->setObjectName(QStringLiteral("dispEnableIMU"));
        dispEnableIMU->setGeometry(QRect(540, 160, 141, 22));
        labelDisplay = new QLabel(centralWidget);
        labelDisplay->setObjectName(QStringLiteral("labelDisplay"));
        labelDisplay->setGeometry(QRect(510, 0, 121, 17));
        QFont font1;
        font1.setBold(true);
        font1.setWeight(75);
        labelDisplay->setFont(font1);
        line_h1 = new QFrame(centralWidget);
        line_h1->setObjectName(QStringLiteral("line_h1"));
        line_h1->setGeometry(QRect(430, 280, 281, 16));
        line_h1->setFrameShape(QFrame::HLine);
        line_h1->setFrameShadow(QFrame::Sunken);
        labelControls = new QLabel(centralWidget);
        labelControls->setObjectName(QStringLiteral("labelControls"));
        labelControls->setGeometry(QRect(520, 300, 121, 17));
        labelControls->setFont(font1);
        noGyro = new QCheckBox(centralWidget);
        noGyro->setObjectName(QStringLiteral("noGyro"));
        noGyro->setGeometry(QRect(730, 40, 97, 22));
        noAccl = new QCheckBox(centralWidget);
        noAccl->setObjectName(QStringLiteral("noAccl"));
        noAccl->setGeometry(QRect(840, 40, 97, 22));
        noMagn = new QCheckBox(centralWidget);
        noMagn->setObjectName(QStringLiteral("noMagn"));
        noMagn->setGeometry(QRect(940, 40, 97, 22));
        viewUp = new QPushButton(centralWidget);
        viewUp->setObjectName(QStringLiteral("viewUp"));
        viewUp->setGeometry(QRect(430, 240, 71, 27));
        viewSide1 = new QPushButton(centralWidget);
        viewSide1->setObjectName(QStringLiteral("viewSide1"));
        viewSide1->setGeometry(QRect(530, 240, 71, 27));
        viewSide2 = new QPushButton(centralWidget);
        viewSide2->setObjectName(QStringLiteral("viewSide2"));
        viewSide2->setGeometry(QRect(630, 240, 71, 27));
        up0 = new QLineEdit(centralWidget);
        up0->setObjectName(QStringLiteral("up0"));
        up0->setGeometry(QRect(480, 330, 71, 27));
        labelUp = new QLabel(centralWidget);
        labelUp->setObjectName(QStringLiteral("labelUp"));
        labelUp->setGeometry(QRect(440, 335, 31, 17));
        labelUp->setFont(font);
        up1 = new QLineEdit(centralWidget);
        up1->setObjectName(QStringLiteral("up1"));
        up1->setGeometry(QRect(560, 330, 71, 27));
        up2 = new QLineEdit(centralWidget);
        up2->setObjectName(QStringLiteral("up2"));
        up2->setGeometry(QRect(640, 330, 71, 27));
        selectReference = new QPushButton(centralWidget);
        selectReference->setObjectName(QStringLiteral("selectReference"));
        selectReference->setGeometry(QRect(430, 370, 111, 27));
        selectReset = new QPushButton(centralWidget);
        selectReset->setObjectName(QStringLiteral("selectReset"));
        selectReset->setGeometry(QRect(550, 370, 61, 27));
        selectDeadRecon = new QPushButton(centralWidget);
        selectDeadRecon->setObjectName(QStringLiteral("selectDeadRecon"));
        selectDeadRecon->setGeometry(QRect(620, 370, 91, 27));
        gBias0 = new QLineEdit(centralWidget);
        gBias0->setObjectName(QStringLiteral("gBias0"));
        gBias0->setGeometry(QRect(1070, 50, 101, 27));
        gBias1 = new QLineEdit(centralWidget);
        gBias1->setObjectName(QStringLiteral("gBias1"));
        gBias1->setGeometry(QRect(1180, 50, 101, 27));
        gBias2 = new QLineEdit(centralWidget);
        gBias2->setObjectName(QStringLiteral("gBias2"));
        gBias2->setGeometry(QRect(1290, 50, 101, 27));
        gMult1 = new QLineEdit(centralWidget);
        gMult1->setObjectName(QStringLiteral("gMult1"));
        gMult1->setGeometry(QRect(1180, 110, 101, 27));
        gMult2 = new QLineEdit(centralWidget);
        gMult2->setObjectName(QStringLiteral("gMult2"));
        gMult2->setGeometry(QRect(1290, 110, 101, 27));
        gMult0 = new QLineEdit(centralWidget);
        gMult0->setObjectName(QStringLiteral("gMult0"));
        gMult0->setGeometry(QRect(1070, 110, 101, 27));
        gMult5 = new QLineEdit(centralWidget);
        gMult5->setObjectName(QStringLiteral("gMult5"));
        gMult5->setGeometry(QRect(1290, 140, 101, 27));
        gMult3 = new QLineEdit(centralWidget);
        gMult3->setObjectName(QStringLiteral("gMult3"));
        gMult3->setGeometry(QRect(1070, 140, 101, 27));
        gMult4 = new QLineEdit(centralWidget);
        gMult4->setObjectName(QStringLiteral("gMult4"));
        gMult4->setGeometry(QRect(1180, 140, 101, 27));
        gMult8 = new QLineEdit(centralWidget);
        gMult8->setObjectName(QStringLiteral("gMult8"));
        gMult8->setGeometry(QRect(1290, 170, 101, 27));
        gMult6 = new QLineEdit(centralWidget);
        gMult6->setObjectName(QStringLiteral("gMult6"));
        gMult6->setGeometry(QRect(1070, 170, 101, 27));
        gMult7 = new QLineEdit(centralWidget);
        gMult7->setObjectName(QStringLiteral("gMult7"));
        gMult7->setGeometry(QRect(1180, 170, 101, 27));
        labelCalib1 = new QLabel(centralWidget);
        labelCalib1->setObjectName(QStringLiteral("labelCalib1"));
        labelCalib1->setGeometry(QRect(1170, 30, 121, 20));
        labelCalib1->setFont(font);
        labelCalib2 = new QLabel(centralWidget);
        labelCalib2->setObjectName(QStringLiteral("labelCalib2"));
        labelCalib2->setGeometry(QRect(1110, 90, 231, 20));
        labelCalib2->setFont(font);
        aBias1 = new QLineEdit(centralWidget);
        aBias1->setObjectName(QStringLiteral("aBias1"));
        aBias1->setGeometry(QRect(1180, 240, 101, 27));
        aMult2 = new QLineEdit(centralWidget);
        aMult2->setObjectName(QStringLiteral("aMult2"));
        aMult2->setGeometry(QRect(1290, 300, 101, 27));
        aMult7 = new QLineEdit(centralWidget);
        aMult7->setObjectName(QStringLiteral("aMult7"));
        aMult7->setGeometry(QRect(1180, 360, 101, 27));
        aMult1 = new QLineEdit(centralWidget);
        aMult1->setObjectName(QStringLiteral("aMult1"));
        aMult1->setGeometry(QRect(1180, 300, 101, 27));
        aMult0 = new QLineEdit(centralWidget);
        aMult0->setObjectName(QStringLiteral("aMult0"));
        aMult0->setGeometry(QRect(1070, 300, 101, 27));
        aMult3 = new QLineEdit(centralWidget);
        aMult3->setObjectName(QStringLiteral("aMult3"));
        aMult3->setGeometry(QRect(1070, 330, 101, 27));
        labelCalib4 = new QLabel(centralWidget);
        labelCalib4->setObjectName(QStringLiteral("labelCalib4"));
        labelCalib4->setGeometry(QRect(1100, 280, 271, 20));
        labelCalib4->setFont(font);
        aMult8 = new QLineEdit(centralWidget);
        aMult8->setObjectName(QStringLiteral("aMult8"));
        aMult8->setGeometry(QRect(1290, 360, 101, 27));
        aBias2 = new QLineEdit(centralWidget);
        aBias2->setObjectName(QStringLiteral("aBias2"));
        aBias2->setGeometry(QRect(1290, 240, 101, 27));
        aBias0 = new QLineEdit(centralWidget);
        aBias0->setObjectName(QStringLiteral("aBias0"));
        aBias0->setGeometry(QRect(1070, 240, 101, 27));
        aMult5 = new QLineEdit(centralWidget);
        aMult5->setObjectName(QStringLiteral("aMult5"));
        aMult5->setGeometry(QRect(1290, 330, 101, 27));
        aMult6 = new QLineEdit(centralWidget);
        aMult6->setObjectName(QStringLiteral("aMult6"));
        aMult6->setGeometry(QRect(1070, 360, 101, 27));
        labelCalib3 = new QLabel(centralWidget);
        labelCalib3->setObjectName(QStringLiteral("labelCalib3"));
        labelCalib3->setGeometry(QRect(1160, 220, 161, 20));
        labelCalib3->setFont(font);
        aMult4 = new QLineEdit(centralWidget);
        aMult4->setObjectName(QStringLiteral("aMult4"));
        aMult4->setGeometry(QRect(1180, 330, 101, 27));
        mBias1 = new QLineEdit(centralWidget);
        mBias1->setObjectName(QStringLiteral("mBias1"));
        mBias1->setGeometry(QRect(1180, 430, 101, 27));
        mMult3 = new QLineEdit(centralWidget);
        mMult3->setObjectName(QStringLiteral("mMult3"));
        mMult3->setGeometry(QRect(1070, 520, 101, 27));
        mMult2 = new QLineEdit(centralWidget);
        mMult2->setObjectName(QStringLiteral("mMult2"));
        mMult2->setGeometry(QRect(1290, 490, 101, 27));
        mMult6 = new QLineEdit(centralWidget);
        mMult6->setObjectName(QStringLiteral("mMult6"));
        mMult6->setGeometry(QRect(1070, 550, 101, 27));
        mMult1 = new QLineEdit(centralWidget);
        mMult1->setObjectName(QStringLiteral("mMult1"));
        mMult1->setGeometry(QRect(1180, 490, 101, 27));
        labelCalib6 = new QLabel(centralWidget);
        labelCalib6->setObjectName(QStringLiteral("labelCalib6"));
        labelCalib6->setGeometry(QRect(1100, 470, 261, 20));
        labelCalib6->setFont(font);
        mBias0 = new QLineEdit(centralWidget);
        mBias0->setObjectName(QStringLiteral("mBias0"));
        mBias0->setGeometry(QRect(1070, 430, 101, 27));
        mMult0 = new QLineEdit(centralWidget);
        mMult0->setObjectName(QStringLiteral("mMult0"));
        mMult0->setGeometry(QRect(1070, 490, 101, 27));
        mBias2 = new QLineEdit(centralWidget);
        mBias2->setObjectName(QStringLiteral("mBias2"));
        mBias2->setGeometry(QRect(1290, 430, 101, 27));
        mMult4 = new QLineEdit(centralWidget);
        mMult4->setObjectName(QStringLiteral("mMult4"));
        mMult4->setGeometry(QRect(1180, 520, 101, 27));
        labelCalib5 = new QLabel(centralWidget);
        labelCalib5->setObjectName(QStringLiteral("labelCalib5"));
        labelCalib5->setGeometry(QRect(1150, 410, 151, 20));
        labelCalib5->setFont(font);
        mMult8 = new QLineEdit(centralWidget);
        mMult8->setObjectName(QStringLiteral("mMult8"));
        mMult8->setGeometry(QRect(1290, 550, 101, 27));
        mMult5 = new QLineEdit(centralWidget);
        mMult5->setObjectName(QStringLiteral("mMult5"));
        mMult5->setGeometry(QRect(1290, 520, 101, 27));
        mMult7 = new QLineEdit(centralWidget);
        mMult7->setObjectName(QStringLiteral("mMult7"));
        mMult7->setGeometry(QRect(1180, 550, 101, 27));
        line_h2 = new QFrame(centralWidget);
        line_h2->setObjectName(QStringLiteral("line_h2"));
        line_h2->setGeometry(QRect(430, 400, 281, 20));
        line_h2->setFrameShape(QFrame::HLine);
        line_h2->setFrameShadow(QFrame::Sunken);
        gMag = new QLineEdit(centralWidget);
        gMag->setObjectName(QStringLiteral("gMag"));
        gMag->setGeometry(QRect(1180, 620, 101, 27));
        aMag = new QLineEdit(centralWidget);
        aMag->setObjectName(QStringLiteral("aMag"));
        aMag->setGeometry(QRect(1070, 620, 101, 27));
        labelCalib7 = new QLabel(centralWidget);
        labelCalib7->setObjectName(QStringLiteral("labelCalib7"));
        labelCalib7->setGeometry(QRect(1080, 600, 81, 20));
        labelCalib7->setFont(font);
        gAng = new QLineEdit(centralWidget);
        gAng->setObjectName(QStringLiteral("gAng"));
        gAng->setGeometry(QRect(1290, 620, 101, 27));
        labelCalib8 = new QLabel(centralWidget);
        labelCalib8->setObjectName(QStringLiteral("labelCalib8"));
        labelCalib8->setGeometry(QRect(1200, 600, 71, 20));
        labelCalib8->setFont(font);
        labelCalib9 = new QLabel(centralWidget);
        labelCalib9->setObjectName(QStringLiteral("labelCalib9"));
        labelCalib9->setGeometry(QRect(1310, 600, 71, 20));
        labelCalib9->setFont(font);
        labelConfig1 = new QLabel(centralWidget);
        labelConfig1->setObjectName(QStringLiteral("labelConfig1"));
        labelConfig1->setGeometry(QRect(730, 140, 121, 20));
        labelConfig1->setFont(font);
        labelConfig = new QLabel(centralWidget);
        labelConfig->setObjectName(QStringLiteral("labelConfig"));
        labelConfig->setGeometry(QRect(790, 0, 171, 20));
        labelConfig->setFont(font1);
        line_v2 = new QFrame(centralWidget);
        line_v2->setObjectName(QStringLiteral("line_v2"));
        line_v2->setGeometry(QRect(1050, 0, 20, 701));
        line_v2->setFrameShape(QFrame::VLine);
        line_v2->setFrameShadow(QFrame::Sunken);
        labelGyroBias = new QLabel(centralWidget);
        labelGyroBias->setObjectName(QStringLiteral("labelGyroBias"));
        labelGyroBias->setGeometry(QRect(10, 430, 81, 20));
        labelGyroBias->setFont(font);
        labelGyroScale = new QLabel(centralWidget);
        labelGyroScale->setObjectName(QStringLiteral("labelGyroScale"));
        labelGyroScale->setGeometry(QRect(10, 450, 81, 20));
        labelGyroScale->setFont(font);
        labelAcclScale = new QLabel(centralWidget);
        labelAcclScale->setObjectName(QStringLiteral("labelAcclScale"));
        labelAcclScale->setGeometry(QRect(10, 500, 81, 20));
        labelAcclScale->setFont(font);
        labelAcclBias = new QLabel(centralWidget);
        labelAcclBias->setObjectName(QStringLiteral("labelAcclBias"));
        labelAcclBias->setGeometry(QRect(10, 480, 81, 20));
        labelAcclBias->setFont(font);
        labelMagnBias = new QLabel(centralWidget);
        labelMagnBias->setObjectName(QStringLiteral("labelMagnBias"));
        labelMagnBias->setGeometry(QRect(10, 550, 81, 20));
        labelMagnBias->setFont(font);
        labelAcclMag = new QLabel(centralWidget);
        labelAcclMag->setObjectName(QStringLiteral("labelAcclMag"));
        labelAcclMag->setGeometry(QRect(10, 520, 81, 20));
        labelAcclMag->setFont(font);
        labelMagnMag = new QLabel(centralWidget);
        labelMagnMag->setObjectName(QStringLiteral("labelMagnMag"));
        labelMagnMag->setGeometry(QRect(10, 570, 81, 20));
        labelMagnMag->setFont(font);
        labelMagnAng = new QLabel(centralWidget);
        labelMagnAng->setObjectName(QStringLiteral("labelMagnAng"));
        labelMagnAng->setGeometry(QRect(10, 590, 81, 20));
        labelMagnAng->setFont(font);
        textMagnBias = new QLabel(centralWidget);
        textMagnBias->setObjectName(QStringLiteral("textMagnBias"));
        textMagnBias->setGeometry(QRect(90, 550, 191, 20));
        textMagnBias->setFont(font);
        textAcclMag = new QLabel(centralWidget);
        textAcclMag->setObjectName(QStringLiteral("textAcclMag"));
        textAcclMag->setGeometry(QRect(90, 520, 191, 20));
        textAcclMag->setFont(font);
        textGyroScale = new QLabel(centralWidget);
        textGyroScale->setObjectName(QStringLiteral("textGyroScale"));
        textGyroScale->setGeometry(QRect(90, 450, 191, 20));
        textGyroScale->setFont(font);
        textAcclScale = new QLabel(centralWidget);
        textAcclScale->setObjectName(QStringLiteral("textAcclScale"));
        textAcclScale->setGeometry(QRect(90, 500, 191, 20));
        textAcclScale->setFont(font);
        textAcclBias = new QLabel(centralWidget);
        textAcclBias->setObjectName(QStringLiteral("textAcclBias"));
        textAcclBias->setGeometry(QRect(90, 480, 191, 20));
        textAcclBias->setFont(font);
        textMagnMag = new QLabel(centralWidget);
        textMagnMag->setObjectName(QStringLiteral("textMagnMag"));
        textMagnMag->setGeometry(QRect(90, 570, 191, 20));
        textMagnMag->setFont(font);
        textMagnAng = new QLabel(centralWidget);
        textMagnAng->setObjectName(QStringLiteral("textMagnAng"));
        textMagnAng->setGeometry(QRect(90, 590, 191, 20));
        textMagnAng->setFont(font);
        textGyroBias = new QLabel(centralWidget);
        textGyroBias->setObjectName(QStringLiteral("textGyroBias"));
        textGyroBias->setGeometry(QRect(90, 430, 191, 20));
        textGyroBias->setFont(font);
        widget = new GLWidget(centralWidget);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(10, 0, 411, 411));
        labelConfig_2 = new QLabel(centralWidget);
        labelConfig_2->setObjectName(QStringLiteral("labelConfig_2"));
        labelConfig_2->setGeometry(QRect(1150, 0, 151, 20));
        labelConfig_2->setFont(font1);
        calibOpen = new QPushButton(centralWidget);
        calibOpen->setObjectName(QStringLiteral("calibOpen"));
        calibOpen->setGeometry(QRect(1130, 670, 81, 27));
        calibSave = new QPushButton(centralWidget);
        calibSave->setObjectName(QStringLiteral("calibSave"));
        calibSave->setGeometry(QRect(1250, 670, 81, 27));
        noWeight = new QCheckBox(centralWidget);
        noWeight->setObjectName(QStringLiteral("noWeight"));
        noWeight->setGeometry(QRect(730, 70, 97, 22));
        noTear = new QCheckBox(centralWidget);
        noTear->setObjectName(QStringLiteral("noTear"));
        noTear->setGeometry(QRect(840, 70, 97, 22));
        noMove = new QCheckBox(centralWidget);
        noMove->setObjectName(QStringLiteral("noMove"));
        noMove->setGeometry(QRect(940, 70, 97, 22));
        noAutocal = new QCheckBox(centralWidget);
        noAutocal->setObjectName(QStringLiteral("noAutocal"));
        noAutocal->setGeometry(QRect(940, 100, 101, 22));
        noFOM = new QCheckBox(centralWidget);
        noFOM->setObjectName(QStringLiteral("noFOM"));
        noFOM->setGeometry(QRect(730, 100, 101, 22));
        noFltr = new QCheckBox(centralWidget);
        noFltr->setObjectName(QStringLiteral("noFltr"));
        noFltr->setGeometry(QRect(840, 100, 91, 22));
        gThreshVal = new QLineEdit(centralWidget);
        gThreshVal->setObjectName(QStringLiteral("gThreshVal"));
        gThreshVal->setGeometry(QRect(730, 160, 151, 27));
        gThreshTime = new QLineEdit(centralWidget);
        gThreshTime->setObjectName(QStringLiteral("gThreshTime"));
        gThreshTime->setGeometry(QRect(900, 160, 151, 27));
        labelConfig2 = new QLabel(centralWidget);
        labelConfig2->setObjectName(QStringLiteral("labelConfig2"));
        labelConfig2->setGeometry(QRect(900, 140, 121, 20));
        labelConfig2->setFont(font);
        labelConfig4 = new QLabel(centralWidget);
        labelConfig4->setObjectName(QStringLiteral("labelConfig4"));
        labelConfig4->setGeometry(QRect(900, 200, 121, 20));
        labelConfig4->setFont(font);
        labelConfig3 = new QLabel(centralWidget);
        labelConfig3->setObjectName(QStringLiteral("labelConfig3"));
        labelConfig3->setGeometry(QRect(730, 200, 121, 20));
        labelConfig3->setFont(font);
        aWeight = new QLineEdit(centralWidget);
        aWeight->setObjectName(QStringLiteral("aWeight"));
        aWeight->setGeometry(QRect(730, 220, 151, 27));
        aAlpha = new QLineEdit(centralWidget);
        aAlpha->setObjectName(QStringLiteral("aAlpha"));
        aAlpha->setGeometry(QRect(900, 220, 151, 27));
        mAlpha = new QLineEdit(centralWidget);
        mAlpha->setObjectName(QStringLiteral("mAlpha"));
        mAlpha->setGeometry(QRect(900, 280, 151, 27));
        labelConfig6 = new QLabel(centralWidget);
        labelConfig6->setObjectName(QStringLiteral("labelConfig6"));
        labelConfig6->setGeometry(QRect(900, 260, 121, 20));
        labelConfig6->setFont(font);
        mWeight = new QLineEdit(centralWidget);
        mWeight->setObjectName(QStringLiteral("mWeight"));
        mWeight->setGeometry(QRect(730, 280, 151, 27));
        labelConfig5 = new QLabel(centralWidget);
        labelConfig5->setObjectName(QStringLiteral("labelConfig5"));
        labelConfig5->setGeometry(QRect(730, 260, 121, 20));
        labelConfig5->setFont(font);
        autocalAlpha = new QLineEdit(centralWidget);
        autocalAlpha->setObjectName(QStringLiteral("autocalAlpha"));
        autocalAlpha->setGeometry(QRect(900, 340, 151, 27));
        labelConfig8 = new QLabel(centralWidget);
        labelConfig8->setObjectName(QStringLiteral("labelConfig8"));
        labelConfig8->setGeometry(QRect(900, 320, 121, 20));
        labelConfig8->setFont(font);
        configSave = new QPushButton(centralWidget);
        configSave->setObjectName(QStringLiteral("configSave"));
        configSave->setGeometry(QRect(910, 670, 81, 27));
        configOpen = new QPushButton(centralWidget);
        configOpen->setObjectName(QStringLiteral("configOpen"));
        configOpen->setGeometry(QRect(790, 670, 81, 27));
        dispVectRaw = new QRadioButton(centralWidget);
        dispVectRaw->setObjectName(QStringLiteral("dispVectRaw"));
        dispVectRaw->setGeometry(QRect(430, 200, 81, 22));
        dispVectCor = new QRadioButton(centralWidget);
        dispVectCor->setObjectName(QStringLiteral("dispVectCor"));
        dispVectCor->setGeometry(QRect(510, 200, 111, 22));
        dispVectFltr = new QRadioButton(centralWidget);
        dispVectFltr->setObjectName(QStringLiteral("dispVectFltr"));
        dispVectFltr->setGeometry(QRect(620, 200, 81, 22));
        labelConfig7 = new QLabel(centralWidget);
        labelConfig7->setObjectName(QStringLiteral("labelConfig7"));
        labelConfig7->setGeometry(QRect(730, 320, 121, 20));
        labelConfig7->setFont(font);
        moveAlpha = new QLineEdit(centralWidget);
        moveAlpha->setObjectName(QStringLiteral("moveAlpha"));
        moveAlpha->setGeometry(QRect(730, 340, 151, 27));
        windowGUI->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(windowGUI);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1400, 25));
        windowGUI->setMenuBar(menuBar);
        mainToolBar = new QToolBar(windowGUI);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        windowGUI->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(windowGUI);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        windowGUI->setStatusBar(statusBar);

        retranslateUi(windowGUI);

        QMetaObject::connectSlotsByName(windowGUI);
    } // setupUi

    void retranslateUi(QMainWindow *windowGUI)
    {
        windowGUI->setWindowTitle(QApplication::translate("windowGUI", "Real Time Display Tool", 0));
        dispScaleGyro->setText(QApplication::translate("windowGUI", "1000", 0));
        dispScaleAccl->setText(QApplication::translate("windowGUI", "2500", 0));
        dispScaleMagn->setText(QApplication::translate("windowGUI", "5000", 0));
        dispScaleIMU->setText(QApplication::translate("windowGUI", "1000", 0));
        dispEnableGyro->setText(QApplication::translate("windowGUI", "gyroscope", 0));
        dispEnableAccl->setText(QApplication::translate("windowGUI", "accelerometer", 0));
        dispEnableMagn->setText(QApplication::translate("windowGUI", "magnetometer", 0));
        dispEnableIMU->setText(QApplication::translate("windowGUI", "IMU", 0));
        labelDisplay->setText(QApplication::translate("windowGUI", "Display Options", 0));
        labelControls->setText(QApplication::translate("windowGUI", "IMU Controls", 0));
        noGyro->setText(QApplication::translate("windowGUI", "no gyro", 0));
        noAccl->setText(QApplication::translate("windowGUI", "no accl", 0));
        noMagn->setText(QApplication::translate("windowGUI", "no magn", 0));
        viewUp->setText(QApplication::translate("windowGUI", "up", 0));
        viewSide1->setText(QApplication::translate("windowGUI", "side 1", 0));
        viewSide2->setText(QApplication::translate("windowGUI", "side 2", 0));
        up0->setText(QApplication::translate("windowGUI", "0", 0));
        labelUp->setText(QApplication::translate("windowGUI", "up", 0));
        up1->setText(QApplication::translate("windowGUI", "0", 0));
        up2->setText(QApplication::translate("windowGUI", "1", 0));
        selectReference->setText(QApplication::translate("windowGUI", "set reference", 0));
        selectReset->setText(QApplication::translate("windowGUI", "reset", 0));
        selectDeadRecon->setText(QApplication::translate("windowGUI", "dead recon", 0));
        gBias0->setText(QApplication::translate("windowGUI", "0.0", 0));
        gBias1->setText(QApplication::translate("windowGUI", "0.0", 0));
        gBias2->setText(QApplication::translate("windowGUI", "0.0", 0));
        gMult1->setText(QApplication::translate("windowGUI", "0.0", 0));
        gMult2->setText(QApplication::translate("windowGUI", "0.0", 0));
        gMult0->setText(QApplication::translate("windowGUI", "1.0", 0));
        gMult5->setText(QApplication::translate("windowGUI", "0.0", 0));
        gMult3->setText(QApplication::translate("windowGUI", "0.0", 0));
        gMult4->setText(QApplication::translate("windowGUI", "1.0", 0));
        gMult8->setText(QApplication::translate("windowGUI", "1.0", 0));
        gMult6->setText(QApplication::translate("windowGUI", "0.0", 0));
        gMult7->setText(QApplication::translate("windowGUI", "0.0", 0));
        labelCalib1->setText(QApplication::translate("windowGUI", "gyroscope biases", 0));
        labelCalib2->setText(QApplication::translate("windowGUI", "gyroscope transformation matrix", 0));
        aBias1->setText(QApplication::translate("windowGUI", "0.0", 0));
        aMult2->setText(QApplication::translate("windowGUI", "0.0", 0));
        aMult7->setText(QApplication::translate("windowGUI", "0.0", 0));
        aMult1->setText(QApplication::translate("windowGUI", "0.0", 0));
        aMult0->setText(QApplication::translate("windowGUI", "1.0", 0));
        aMult3->setText(QApplication::translate("windowGUI", "0.0", 0));
        labelCalib4->setText(QApplication::translate("windowGUI", "accelerometer transformation matrix", 0));
        aMult8->setText(QApplication::translate("windowGUI", "1.0", 0));
        aBias2->setText(QApplication::translate("windowGUI", "0.0", 0));
        aBias0->setText(QApplication::translate("windowGUI", "0.0", 0));
        aMult5->setText(QApplication::translate("windowGUI", "0.0", 0));
        aMult6->setText(QApplication::translate("windowGUI", "0.0", 0));
        labelCalib3->setText(QApplication::translate("windowGUI", "accelerometer biases", 0));
        aMult4->setText(QApplication::translate("windowGUI", "1.0", 0));
        mBias1->setText(QApplication::translate("windowGUI", "0.0", 0));
        mMult3->setText(QApplication::translate("windowGUI", "0.0", 0));
        mMult2->setText(QApplication::translate("windowGUI", "0.0", 0));
        mMult6->setText(QApplication::translate("windowGUI", "0.0", 0));
        mMult1->setText(QApplication::translate("windowGUI", "0.0", 0));
        labelCalib6->setText(QApplication::translate("windowGUI", "magnetometer transformation matrix", 0));
        mBias0->setText(QApplication::translate("windowGUI", "0.0", 0));
        mMult0->setText(QApplication::translate("windowGUI", "1.0", 0));
        mBias2->setText(QApplication::translate("windowGUI", "0.0", 0));
        mMult4->setText(QApplication::translate("windowGUI", "1.0", 0));
        labelCalib5->setText(QApplication::translate("windowGUI", "magnetometer biases", 0));
        mMult8->setText(QApplication::translate("windowGUI", "1.0", 0));
        mMult5->setText(QApplication::translate("windowGUI", "0.0", 0));
        mMult7->setText(QApplication::translate("windowGUI", "0.0", 0));
        gMag->setText(QApplication::translate("windowGUI", "0.0", 0));
        aMag->setText(QApplication::translate("windowGUI", "0.0", 0));
        labelCalib7->setText(QApplication::translate("windowGUI", "gravity mag", 0));
        gAng->setText(QApplication::translate("windowGUI", "0.0", 0));
        labelCalib8->setText(QApplication::translate("windowGUI", "field mag", 0));
        labelCalib9->setText(QApplication::translate("windowGUI", "field ang", 0));
        labelConfig1->setText(QApplication::translate("windowGUI", "gThreshVal", 0));
        labelConfig->setText(QApplication::translate("windowGUI", "Configuration Structure", 0));
        labelGyroBias->setText(QApplication::translate("windowGUI", "gyro bias:", 0));
        labelGyroScale->setText(QApplication::translate("windowGUI", "gyro scale:", 0));
        labelAcclScale->setText(QApplication::translate("windowGUI", "accl scale:", 0));
        labelAcclBias->setText(QApplication::translate("windowGUI", "accl bias:", 0));
        labelMagnBias->setText(QApplication::translate("windowGUI", "magn bias:", 0));
        labelAcclMag->setText(QApplication::translate("windowGUI", "accl mag:", 0));
        labelMagnMag->setText(QApplication::translate("windowGUI", "magn mag:", 0));
        labelMagnAng->setText(QApplication::translate("windowGUI", "magn ang:", 0));
        textMagnBias->setText(QApplication::translate("windowGUI", "-", 0));
        textAcclMag->setText(QApplication::translate("windowGUI", "-", 0));
        textGyroScale->setText(QApplication::translate("windowGUI", "-", 0));
        textAcclScale->setText(QApplication::translate("windowGUI", "-", 0));
        textAcclBias->setText(QApplication::translate("windowGUI", "-", 0));
        textMagnMag->setText(QApplication::translate("windowGUI", "-", 0));
        textMagnAng->setText(QApplication::translate("windowGUI", "-", 0));
        textGyroBias->setText(QApplication::translate("windowGUI", "-", 0));
        labelConfig_2->setText(QApplication::translate("windowGUI", "Calibration Structure", 0));
        calibOpen->setText(QApplication::translate("windowGUI", "Open", 0));
        calibSave->setText(QApplication::translate("windowGUI", "Save", 0));
        noWeight->setText(QApplication::translate("windowGUI", "no weight", 0));
        noTear->setText(QApplication::translate("windowGUI", "no tear", 0));
        noMove->setText(QApplication::translate("windowGUI", "no move", 0));
        noAutocal->setText(QApplication::translate("windowGUI", "no autocal", 0));
        noFOM->setText(QApplication::translate("windowGUI", "no FOM", 0));
        noFltr->setText(QApplication::translate("windowGUI", "no fltr", 0));
        gThreshVal->setText(QApplication::translate("windowGUI", "1.0", 0));
        gThreshTime->setText(QApplication::translate("windowGUI", "1.0", 0));
        labelConfig2->setText(QApplication::translate("windowGUI", "gThreshTime", 0));
        labelConfig4->setText(QApplication::translate("windowGUI", "aAlpha", 0));
        labelConfig3->setText(QApplication::translate("windowGUI", "aWeight", 0));
        aWeight->setText(QApplication::translate("windowGUI", "1.0", 0));
        aAlpha->setText(QApplication::translate("windowGUI", "1.0", 0));
        mAlpha->setText(QApplication::translate("windowGUI", "1.0", 0));
        labelConfig6->setText(QApplication::translate("windowGUI", "mAlpha", 0));
        mWeight->setText(QApplication::translate("windowGUI", "1.0", 0));
        labelConfig5->setText(QApplication::translate("windowGUI", "mWeight", 0));
        autocalAlpha->setText(QApplication::translate("windowGUI", "1.0", 0));
        labelConfig8->setText(QApplication::translate("windowGUI", "autocalAlpha", 0));
        configSave->setText(QApplication::translate("windowGUI", "Save", 0));
        configOpen->setText(QApplication::translate("windowGUI", "Open", 0));
        dispVectRaw->setText(QApplication::translate("windowGUI", "Raw", 0));
        dispVectCor->setText(QApplication::translate("windowGUI", "Corrected", 0));
        dispVectFltr->setText(QApplication::translate("windowGUI", "Filtered", 0));
        labelConfig7->setText(QApplication::translate("windowGUI", "moveAlpha", 0));
        moveAlpha->setText(QApplication::translate("windowGUI", "1.0", 0));
    } // retranslateUi

};

namespace Ui {
    class windowGUI: public Ui_windowGUI {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WINDOWGUI_H

QT               += core                \
                    gui                 \
                    opengl              \
                    widgets

QMAKE_RPATHDIR   += ../bin

OBJECTS_DIR       = ./obj

HEADERS          += glWidget.h          \
                    windowGUI.h         \
                    configGUI.h         \
                    dataIF.h

SOURCES          += glWidget.cpp        \
                    windowGUI.cpp       \
                    configGUI.cpp       \ 
                    dataIF.c            \
                    main.cpp 

VPATH            += ../common

INCLUDEPATH      += ../imu              \
                    ../common

FORMS            += windowGUI.ui

LIBS             += -L../bin            \
                    -lIMU               \ 
                    -lGLU               \
                    -lcsv               \
                    -lpthread

DESTDIR           = ../bin

DEFINES          += IMU_TYPE=${IMU_TYPE} 

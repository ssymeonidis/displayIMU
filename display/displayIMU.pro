QT               += core                \
                    gui                 \
                    opengl              \
                    widgets

QMAKE_RPATHDIR   += ../bin

OBJECTS_DIR       = ./obj

HEADERS          += glWidget.h          \
                    windowGUI.h         \
                    dataParse.h

SOURCES          += glWidget.cpp        \
                    windowGUI.cpp       \
                    dataParse.cpp       \
                    main.cpp 

INCLUDEPATH      += ../imu

FORMS            += windowGUI.ui

LIBS             += -L../bin            \
                    -lIMU               \ 
                    -lGLU               \
                    -lcsv               \
                    -lpthread

DESTDIR           = ../bin

DEFINES          += IMU_TYPE=${IMU_TYPE}

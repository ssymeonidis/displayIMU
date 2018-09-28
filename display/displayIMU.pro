QT               += core                \
                    gui                 \
                    opengl              \
                    widgets

QMAKE_RPATHDIR   += ../build

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

LIBS             += -L../build          \
                    -lIMU               \ 
                    -lGLU               \
                    -lcsv               \
                    -lpthread

DESTDIR           = ../build

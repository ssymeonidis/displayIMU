QT           += core                \
                gui                 \
                opengl              \
                widgets

OBJECTS_DIR   = ./obj

HEADERS      += glWidget.h          \
                windowGUI.h         \
                IMU.h               \
                dataParse.h         \
                fileUtils.h

SOURCES      += glWidget.cpp        \
                windowGUI.cpp       \
                IMU.cpp             \
                dataParse.cpp       \
                fileUtils.cpp       \
                main.cpp 

FORMS        += windowGUI.ui

LIBS         += -lGLU  \
                -lcsv  \
                -lpthread

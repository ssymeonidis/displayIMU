VPATH        += ../shared
INCLUDEPATH  += ../shared

HEADERS       = glWidget.h          \
                windowGUI.h         \
                IMU.h               \
                dataParse.h
SOURCES       = glWidget.cpp        \
                windowGUI.cpp       \
                IMU.cpp             \
                dataParse.cpp       \
                main.cpp 
QT           += opengl              \
                widgets
LIBS         += -lGLU  \
                -lcsv  \
                -lpthread

VPATH        += ../shared
INCLUDEPATH  += ../shared

HEADERS       = glwidget.h          \
                window.h            \
                IMU.h               \
                dataParse.h
SOURCES       = glwidget.cpp        \
                window.cpp          \
                IMU.cpp             \
                dataParse.cpp       \
                main.cpp 
QT           += opengl              \
                widgets
LIBS         += -lGLU  \
                -lcsv  \
                -lpthread

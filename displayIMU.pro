VPATH        += ../shared
INCLUDEPATH  += ../shared

HEADERS       = glwidget.h          \
                pixmap.h            \
                window.h            \
                MARG.h              \
                receive.h           \
                dataread.h          \
                framesPerSecond.h
SOURCES       = glwidget.cpp        \
                pixmap.cpp          \
                window.cpp          \
                MARG.cpp            \
                receive.cpp         \
                dataread.cpp        \
                framesPerSecond.cpp \
                main.cpp 
QT           += opengl              \
                widgets
LIBS         += -lGLU  \
                -lcsv  \
                -lpthread

VPATH        += ../shared
INCLUDEPATH  += ../shared

HEADERS       = glwidget.h          \
                window.h            \
                MARG.h              \
                receive.h           \
                dataread.h
SOURCES       = glwidget.cpp        \
                window.cpp          \
                MARG.cpp            \
                receive.cpp         \
                dataread.cpp        \
                main.cpp 
QT           += opengl              \
                widgets
LIBS         += -lGLU  \
                -lcsv  \
                -lpthread

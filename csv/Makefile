CC          = gcc 
CFLAGS      = -fPIC -Wall -Wextra -O2 -g
INCLUDE     = -I../imu -I../common
BINDIR      = ../bin
OBJDIR      = obj
RM          = rm -f
DEFINES     = -D"IMU_TYPE=${IMU_TYPE}"
LIBS        = -L../bin                   \
              -lIMU                      \
              -lm                        \
              -lpthread
LINKER      = -Wl,-rpath=.
TARGETS     = $(BINDIR)/csv_process

all: ${TARGETS}

$(BINDIR)/csv_process: $(OBJDIR)/csv_process.o $(OBJDIR)/dataIF.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(OBJDIR)/csv_process.o: csv_process.c
	$(CC) $(CFLAGS) ${DEFINES} ${INCLUDE} -c $< -o $@

$(OBJDIR)/dataIF.o: ../common/dataIF.c
	$(CC) $(CFLAGS) ${DEFINES} ${INCLUDE} -c $< -o $@

clean:
	-${RM} ${TARGETS} ${OBJS}

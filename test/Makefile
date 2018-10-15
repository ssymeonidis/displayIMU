CC          = gcc 
CFLAGS      = -fPIC -Wall -Wextra -O2 -g 
INCLUDE     = -I../imu
BINDIR      = bin
OBJDIR      = obj
RM          = rm -f  
DEFINES     = -D"IMU_TYPE=${IMU_TYPE}"
TARGETS     = $(BINDIR)/test_datum
LIBS        = -L../bin               \
              -lIMU                  \
              -lpthread
LINKER      = -Wl,-rpath=../../bin
SRCS        = test_datum.c
OBJS        = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: ${TARGETS}

$(TARGETS): $(OBJS)
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) ${DEFINES} ${INCLUDE} -c $< -o $@

clean:
	-${RM} ${TARGETS} ${OBJS}

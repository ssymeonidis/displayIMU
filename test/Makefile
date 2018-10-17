CC          = gcc 
CFLAGS      = -fPIC -Wall -Wextra -O2 -g 
INCLUDE     = -I../imu
BINDIR      = bin
OBJDIR      = obj
RM          = rm -f  
DEFINES     = -D"IMU_TYPE=${IMU_TYPE}"
TARGETS     = $(BINDIR)/test_datum   \
              $(BINDIR)/test_pnts
LIBS        = -L../bin               \
              -lIMU                  \
              -lpthread
LINKER      = -Wl,-rpath=../../bin
SRCS        = test_datum.c           \
              test_pnts.c
OBJS        = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: ${TARGETS}

$(BINDIR)/test_datum: $(OBJDIR)/test_datum.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_pnts:  $(OBJDIR)/test_pnts.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) ${DEFINES} ${INCLUDE} -c $< -o $@

clean:
	-${RM} ${TARGETS} ${OBJS}

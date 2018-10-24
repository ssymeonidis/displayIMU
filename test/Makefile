CC          = gcc 
CFLAGS      = -fPIC -Wall -Wextra -O2 -g 
INCLUDE     = -I../imu
BINDIR      = bin
OBJDIR      = obj
RM          = rm -f  
DEFINES     = -D"IMU_TYPE=${IMU_TYPE}"
TARGETS     = $(BINDIR)/test_datum       \
              $(BINDIR)/test_pnts_gyro   \
              $(BINDIR)/test_euler       \
              $(BINDIR)/test_quat_math   \
              $(BINDIR)/test_estm_accl   \
              $(BINDIR)/test_estm_magn
LIBS        = -L../bin                   \
              -lIMU                      \
              -lpthread
LINKER      = -Wl,-rpath=../../bin
SRCS        = test_datum.c               \
              test_pnts_gyro.c           \
              test_euler.c               \
              test_quat_math.c           \
              test_estm_accl.c           \
              test_estm_magn.c
OBJS        = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: ${TARGETS}

$(BINDIR)/test_datum: $(OBJDIR)/test_datum.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_pnts_gyro: $(OBJDIR)/test_pnts_gyro.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_euler: $(OBJDIR)/test_euler.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_quat_math: $(OBJDIR)/test_quat_math.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_estm_accl: $(OBJDIR)/test_estm_accl.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_estm_magn: $(OBJDIR)/test_estm_magn.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) ${DEFINES} ${INCLUDE} -c $< -o $@

clean:
	-${RM} ${TARGETS} ${OBJS}

run_all:
	cd $(BINDIR); ./test_datum       | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_pnts_gyro   | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_euler       | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_quat_math   | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_estm_accl   | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_estm_magn   | grep -e pass -e error -e fail


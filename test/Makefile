CC          = gcc 
CFLAGS      = -fPIC -Wall -Wextra -O2 -g
INCLUDE     = -I../imu
BINDIR      = bin
OBJDIR      = obj
RM          = rm -f  
DEFINES     = -D"IMU_TYPE=${IMU_TYPE}"
LIBS        = -L../bin                   \
              -lIMU                      \
              -lm                        \
              -lpthread
LINKER      = -Wl,-rpath=../../bin
SRCS        = test_datum.c               \
              test_euler.c               \
              test_quat_math.c           \
              test_estm_gyro.c           \
              test_estm_accl.c           \
              test_estm_magn.c           \
              test_core_gyro.c           \
              test_core_accl.c           \
              test_core_magn.c           \
              test_fom_accl.c            \
              test_fom_magn.c            \
              test_pnts_gyro.c           \
              test_pnts_fnc.c            \
              test_calb_bias.c
OBJS        = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))
TARGETS     = $(patsubst %.c,$(BINDIR)/%,$(SRCS))

all: ${TARGETS}

$(BINDIR)/test_datum: $(OBJDIR)/test_datum.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_euler: $(OBJDIR)/test_euler.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_quat_math: $(OBJDIR)/test_quat_math.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_estm_gyro: $(OBJDIR)/test_estm_gyro.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_estm_accl: $(OBJDIR)/test_estm_accl.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_estm_magn: $(OBJDIR)/test_estm_magn.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_core_gyro: $(OBJDIR)/test_core_gyro.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_core_accl: $(OBJDIR)/test_core_accl.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_core_magn: $(OBJDIR)/test_core_magn.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_fom_accl: $(OBJDIR)/test_fom_accl.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_fom_magn: $(OBJDIR)/test_fom_magn.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_pnts_gyro: $(OBJDIR)/test_pnts_gyro.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_pnts_fnc: $(OBJDIR)/test_pnts_fnc.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(BINDIR)/test_calb_bias: $(OBJDIR)/test_calb_bias.o
	$(CC) ${DEFINES} ${INCLUDE} -o $@ $^ ${LIBS} ${LINKER}

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) ${DEFINES} ${INCLUDE} -c $< -o $@

clean:
	-${RM} ${TARGETS} ${OBJS}

run:
	cd $(BINDIR); ./test_datum     | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_euler     | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_quat_math | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_estm_gyro | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_estm_accl | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_estm_magn | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_core_gyro | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_core_accl | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_core_magn | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_fom_accl  | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_fom_magn  | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_pnts_gyro | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_pnts_fnc  | grep -e pass -e error -e fail
	cd $(BINDIR); ./test_calb_bias | grep -e pass -e error -e fail

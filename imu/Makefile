CC          = gcc 
CFLAGS      = -fPIC -Wall -Wextra -O2 -g 
LDFLAGS     = -shared  
BINDIR      = ../bin
OBJDIR      = obj
RM          = rm -f  
TARGET_LIB  = $(BINDIR)/libIMU.so 
DEFINES     = -D"IMU_MAX_INST=${IMU_MAX_INST}"                 \
              -D"IMU_TYPE=${IMU_TYPE}"                         \
              -D"IMU_CALIB_PNTS_SIZE=${IMU_CALIB_PNTS_SIZE}"   \
              -D"IMU_CALIB_CTRL_SIZE=${IMU_CALIB_CTRL_SIZE}"
SRCS        = IMU_util_file.c   \
              IMU_util_math.c   \
              IMU_correct.c     \
              IMU_calib_pnts.c  \
              IMU_calib_ctrl.c  \
              IMU_calib_auto.c  \
              IMU_core.c 

OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: ${TARGET_LIB}

$(TARGET_LIB): $(OBJS)
	$(CC) ${LDFLAGS} ${DEFINES} -o $@ $^

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) ${DEFINES} -c $< -o $@

clean:
	-${RM} ${TARGET_LIB} ${OBJS}

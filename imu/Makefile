CC          = gcc 
CFLAGS      = -fPIC -Wall -Wextra -O2 -g 
LDFLAGS     = -shared  
BINDIR      = ../bin
OBJDIR      = obj
RM          = rm -f  
TARGET_LIB  = $(BINDIR)/libIMU.so 
DEFINES     = -D"IMU_MAX_INST=${IMU_MAX_INST}"     \
              -D"IMU_TYPE=${IMU_TYPE}"             \
              -D"IMU_PNTS_SIZE=${IMU_PNTS_SIZE}"   \
              -D"IMU_CALB_SIZE=${IMU_CALB_SIZE}"
SRCS        = IMU_file.c  \
              IMU_math.c  \
              IMU_rect.c  \
              IMU_pnts.c  \
              IMU_calb.c  \
              IMU_auto.c  \
              IMU_core.c 

OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: ${TARGET_LIB}

$(TARGET_LIB): $(OBJS)
	$(CC) ${LDFLAGS} ${DEFINES} -o $@ $^

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) ${DEFINES} -c $< -o $@

clean:
	-${RM} ${TARGET_LIB} ${OBJS}

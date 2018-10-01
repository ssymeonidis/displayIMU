CC          = gcc 
CFLAGS      = -fPIC -Wall -Wextra -O2 -g 
LDFLAGS     = -shared  
BINDIR      = ../bin
OBJDIR      = obj
RM          = rm -f  
TARGET_LIB  = $(BINDIR)/libIMU.so 
DEFINES     = -D"IMU_MAX_INST=${IMU_MAX_INST}" \
              -D"IMU_TYPE=${IMU_TYPE}"

SRCS = IMU_util.c IMU_correct.c IMU_core.c 
OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: ${TARGET_LIB}

$(TARGET_LIB): $(OBJS)
	$(CC) ${LDFLAGS} ${DEFINES} -o $@ $^

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) ${DEFINES} -c $< -o $@

clean:
	-${RM} ${TARGET_LIB} ${OBJS}


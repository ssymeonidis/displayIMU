CC          = g++ 
CFLAGS      = -fPIC -Wall -Wextra -O2 -g 
LDFLAGS     = -shared  
BINDIR      = ../bin
OBJDIR      = obj
RM          = rm -f  
TARGET_LIB  = $(BINDIR)/libIMU.so 

SRCS = fileUtils.c IMU.c 
OBJS = $(patsubst %.c,$(OBJDIR)/%.o,$(SRCS))

all: ${TARGET_LIB}

$(TARGET_LIB): $(OBJS)
	$(CC) ${LDFLAGS} -o $@ $^

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	-${RM} ${TARGET_LIB} ${OBJS}


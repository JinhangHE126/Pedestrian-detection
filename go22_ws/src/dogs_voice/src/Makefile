#common makefile header

DIR_INC = /home/unitree/go11_ws/src/dogs_voice/include/:/opt/ros/melodic/include/
DIR_LIB = /home/unitree/go11_ws/src/dogs_voice/libs/:/opt/ros/melodic/lib/
LDFLAGS += -lmsc -lrt -ldl -lpthread -lstdc++ -lroscpp -lrosconsole

TARGET  =demo
BIN_TARGET = $(DIR_BIN)/$(TARGET)

CROSS_COMPILE = 
CFLAGS = -g -Wall -I$(DIR_INC)

ifdef LINUX64
LDFLAGS := -L$(DIR_LIB)/x64
else
LDFLAGS := -L$(DIR_LIB)/x86 
endif
LDFLAGS += -lmsc -lrt -ldl -lpthread -lstdc++

OBJECTS := $(patsubst %.c,%.o,$(wildcard *.c))

$(BIN_TARGET) : $(OBJECTS)
	$(CROSS_COMPILE)gcc $(CFLAGS) $^ -o $@ $(LDFLAGS)

%.o : %.c
	$(CROSS_COMPILE)gcc -c $(CFLAGS) $< -o $@
clean:
	@rm -f *.o $(BIN_TARGET)

.PHONY:clean

#common makefile foot

MYENV = /home/ubuntu/github/swarm_simvp

SRCS = ecbs_without_boost.cpp
SRCS += $(MYENV)/build/opt/runtime/lib/swarm/libsimalloc.a
LIBS =
INCLUDES = -I./

LIBPATH =      $(MYENV)/build/release/runtime/lib/
INCLUDES +=  -I$(MYENV)/runtime/include/ \
						 -I${PORT_DIR}/common
LIBS += -lpthread
CFLAGS   += -std=c++14 -O3 -gdwarf-3 -g -fno-stack-protector -no-pie -mcmodel=large
CFLAGS   += -DSWARM_ALIGNMENT=64  -DSWARM_CACHE_LINE=64 -DSWARM_RUNTIME=1

CC = g++

TARGET = ecbs_simple_swarm

all: ${TARGET}

$(TARGET): $(SRCS)
	@$(CC) $(SRCS) $(CFLAGS) $(LIBS) $(INCLUDES) -o $(TARGET) -lboost_program_options -lyaml-cpp

clean:
	@rm ${TARGET}



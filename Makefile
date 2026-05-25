#

USE_MQTT ?= 0
DEBUG ?= 0

CC       = cc
CXX      = c++
CFLAGS   = -Wall -std=c11 -MMD -pthread -DARM_MATH_RPI
CXXFLAGS = -Wall -std=c++11 -fpermissive -MMD -pthread -DARM_MATH_RPI -fno-strict-aliasing -Wno-c++11-narrowing
LIBS     = -lpthread -lSoapySDR
LDFLAGS  = -L/usr/local/lib

ifeq ($(USE_MQTT), 1)
	CXXFLAGS+= -DUSE_MQTT=1
	LDFLAGS+= -lmosquitto
endif

ifeq ($(DEBUG), 1)
	CFLAGS+= -DDEBUG -g
	CXXFLAGS+= -DDEBUG -g
	LDFLAGS+= -g
else
	CFLAGS+= -DNDEBUG -O3
	CXXFLAGS+= -DNDEBUG -O3
endif

ifeq ($(shell uname -s),Darwin)
	CFLAGS+= -I/opt/homebrew/include
	CXXFLAGS+= -I/opt/homebrew/include
	LDFLAGS+= -L/opt/homebrew/lib
endif

CXXSRCS = $(wildcard *.cpp)
CXXDEPS = $(CXXSRCS:.cpp=.d)
CXXOBJS = $(CXXSRCS:.cpp=.o)

all:		MMDVM-IQ

MMDVM-IQ:	$(CXXOBJS)
		$(CXX) $(CXXOBJS) $(LDFLAGS) $(LIBS) -o MMDVM-IQ

%.o: %.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<
-include $(CXXDEPS)

MMDVM-IQ.o: GitVersion.h FORCE
SerialPort.o: GitVersion.h FORCE

.PHONY: GitVersion.h

FORCE:

install:
		install -m 755 MMDVM-IQ /usr/local/bin/
		install -m 644 MMDVM-IQ.ini /etc

clean:
		$(RM) MMDVM-IQ *.o *.d *.bak *~ GitVersion.h

# Export the current git version if the index file exists, else 000...
GitVersion.h:
ifneq ("$(wildcard .git/index)","")
	echo "const char *gitversion = \"$(shell git rev-parse HEAD)\";" > $@
else
	echo "const char *gitversion = \"0000000000000000000000000000000000000000\";" > $@
endif

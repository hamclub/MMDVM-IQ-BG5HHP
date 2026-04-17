#

# Library paths
DSP_LIB_PATH=./CMSIS_4/CMSIS
DSP_SRC_PATH=$(DSP_LIB_PATH)/DSP_Lib/Source

CC       = cc
CXX      = c++
CFLAGS   = -g -O3 -Wall -std=c11 -MMD -MD -pthread -I$(DSP_LIB_PATH)/Include -DARM_MATH_CM0
CXXFLAGS = -g -O3 -Wall -std=c++11 -fpermissive -MMD -MD -pthread -I$(DSP_LIB_PATH)/Include -DARM_MATH_CM0
LIBS     = -lpthread -lmosquitto -lSoapySDR
LDFLAGS  = -g -L/usr/local/lib

CXXSRCS = $(wildcard *.cpp)
CXXDEPS = $(CXXSRCS:.cpp=.d)
CXXOBJS = $(CXXSRCS:.cpp=.o)

CSRCS = $(wildcard \
 $(DSP_SRC_PATH)/FastMathFunctions/*.c  \
 $(DSP_SRC_PATH)/FilteringFunctions/*.c \
 $(DSP_SRC_PATH)/SupportFunctions/*.c   \
 $(DSP_SRC_PATH)/CommonTables/*.c       )
CDEPS = $(CSRCS:.c=.d)
COBJS = $(CSRCS:.c=.o)

all:		MMDVM-IQ

MMDVM-IQ:	$(CXXOBJS) $(COBJS)
		$(CXX) $(CXXOBJS) $(COBJS) $(LDFLAGS) $(LIBS) -o MMDVM-IQ

%.o: %.cpp
		$(CXX) $(CXXFLAGS) -c -o $@ $<
-include $(CXXDEPS)

%.o: %.c
		$(CC) $(CFLAGS) -c -o $@ $<

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

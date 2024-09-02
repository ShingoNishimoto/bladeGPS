# Makefile for Linux etc.

.PHONY: all clean do_nothing_and_never_up_to_date install
all: libbladegps.so

SHELL=/bin/bash
CC=gcc
CXX=g++
#CFLAGS+=-O3 -Wall -I../../bladeRF/host/libraries/libbladeRF/include -fPIC
CFLAGS+=-g -Wall -I../../bladeRF/host/libraries/libbladeRF/include -fPIC
LDFLAGS=-g -lm -lpthread -L../../bladeRF/host/build/output -lbladeRF -shared

# gnss-sdr cpp files for environment
libs_dir = ../algorithms/libs/environment/
rtklib_dir = ../algorithms/libs/rtklib/
CXXFLAGS =-g -Wall -lm -I$(libs_dir) -I$(rtklib_dir) -I../core/system_parameters/ -I$(HOME)/work/cspice/include -fPIC
LDXXFLAGS=-g -lm -I$(rtklib_dir) -I../core/system_parameters/ -shared
CXX_SRCS = $(libs_dir)time_system.cc $(libs_dir)celestial_body.cc $(libs_dir)earth.cc $(libs_dir)frame.cc $(libs_dir)moon.cc #$(rtklib_dir)rtklib_rtkcmn.cc
CXX_OBJS = $(CXX_SRCS:.cc=.o)

GIT_COMMIT_INFO := $(shell git log -1 | sed -e '1,2s/$$/\\n/;4,$$d' | tr -d '\n')
GIT_COMMIT_ID := $(shell git log -1 | sed -n '1,1p')
GIT_COMMIT_DATE := $(shell git log -1 | sed -n '3,3p')
CFLAGS+=-DGIT_COMMIT_ID="$(GIT_COMMIT_ID)" -DGIT_COMMIT_DATE="$(GIT_COMMIT_DATE)"

# TARGET = environment.so

# all: $(TARGET)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

$(libs_dir)%.o: $(libs_dir)%.cc
	$(CXX) $(CXXFLAGS) -c $< -o $@

# $(rtklib_dir)rtklib_rtkcmn.o: $(rtklib_dir)rtklib_rtkcmn.cc
# 	$(CXX) $(CXXFLAGS) -c $< -o $@

# $(TARGET): $(CXX_OBJS)
# 	$(CXX) $(LDXXFLAGS) -o $@ $(CXX_OBJS)

# libbladegps.so: bladegps.o gpssim.o gpssatellite.o $(TARGET)
libbladegps.so: bladegps.o gpssim.o gpssatellite.o $(CXX_OBJS)
	${CXX} $^ ${LDFLAGS} -o $@ ~/work/cspice/lib/cspice.a
# -L$(libs_dir) -lenvironment

clean:
	rm -f *.o *.so $(libs_dir)*.o

install:
	install libbladegps.so /usr/local/lib

# Makefile for Linux etc.

.PHONY: all clean do_nothing_and_never_up_to_date install
all: libbladegps.so

SHELL=/bin/bash
CC=gcc
#CFLAGS+=-O3 -Wall -I../../bladeRF/host/libraries/libbladeRF/include -fPIC
CFLAGS+=-g -Wall -I../../bladeRF/host/libraries/libbladeRF/include -fPIC
LDFLAGS=-g -lm -lpthread -L../../bladeRF/host/build/output -lbladeRF -shared

GIT_COMMIT_INFO := $(shell git log -1 | sed -e '1,2s/$$/\\n/;4,$$d' | tr -d '\n')
GIT_COMMIT_ID := $(shell git log -1 | sed -n '1,1p')
GIT_COMMIT_DATE := $(shell git log -1 | sed -n '3,3p')
CFLAGS+=-DGIT_COMMIT_ID="$(GIT_COMMIT_ID)" -DGIT_COMMIT_DATE="$(GIT_COMMIT_DATE)"

do_nothing_and_never_up_to_date:
	:

bladegps.o: bladegps.c do_nothing_and_never_up_to_date
	${CC} -c bladegps.c ${CFLAGS} -o bladegps.o

libbladegps.so: bladegps.o gpssim.o gpssatellite.o
	${CC} $^ ${LDFLAGS} -o $@

clean:
	rm -f *.o *.so

install:
	install libbladegps.so /usr/local/lib

#CFLAGS += -g -Wall
CFLAGS += -g -Wall -O3 -march=nocona -fomit-frame-pointer
CFLAGS += -I/usr/include/opencv -DUSE_LIBMORPHO
LDFLAGS += -lCamellia -lcv -lmorpho-v1.2
CC = gcc

cam_dilation : cam_dilation.o

cam_dilation.o : cam_dilation.c

clean:
	rm *.o

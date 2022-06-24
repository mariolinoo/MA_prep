CC = nvcc

PATH_OPENCV = /usr/local/include/opencv4
PATH_JETINF = /usr/local/include/jetson-inference
PATH_JETUTIL = /usr/local/include/jetson-utils
INCPATH = -I ./include -I $(PATH_OPENCV) -I $(PATH_JETINF) -I $(PATH_JETUTIL)

LPATH = -L/usr/local/lib

CFLAGS = --gpu-architecture=compute_53 --gpu-code=compute_53 -O3 $(INCPATH) $(LPATH)
CFLAGSDBG = --gpu-architecture=compute_53 --gpu-code=compute_53 -g $(INCPATH) $(LPATH)
LDFLAGS = -lrealsense2 -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -ljetson-inference -ljetson-utils -lpthread

SRCDIR := src/
INCDIR := include/

SRCS = $(SRCDIR)realsense.cu $(SRCDIR)camera.cu $(SRCDIR)gpio.cu $(SRCDIR)objDet.cu $(SRCDIR)clockRt.cu $(SRCDIR)utils.cu
INCS = $(INCDIR)realsense.h $(INCDIR)global_defines.h $(INCDIR)camera.h $(INCDIR)gpio.h $(INCDIR)objDet.h $(INCDIR)clockRt.h $(INCDIR)utils.h

.PHONY: all clean default debug realsense

default: realsense

all: realsense debug

realsense: $(SRCS) $(INCS)
	$(CC) $(CFLAGS) -o app_realsense $(SRCS) $(LDFLAGS)
	
debug: $(SRCS) $(INCS)
	$(CC) $(CFLAGSDBG) -o app_realsense_dbg $(SRCS) $(LDFLAGS)
	
clean:
	rm -rf app_realsense app_realsense_dbg

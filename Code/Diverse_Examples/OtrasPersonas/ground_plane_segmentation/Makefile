INCDIR = -I.
DBG    = -g
OPT    = -O1 -std=c++0x -pthread
#-O3
#Optimize yet more. -O3 turns on all optimizations specified by -O2 and also turns on the -finline-functions, -funswitch-loops, -fpredictive-commoning, -fgcse-after-reload, -ftree-loop-vectorize, -ftree-slp-vectorize, -fvect-cost-model, -ftree-partial-pre and -fipa-cp-clone options. 
CPP    = g++
#CFLAGS = $(DBG) $(OPT) $(INCDIR) -I/usr/local/include/opencv -I/usr/local/include
CFLAGS =  $(DBG) $(OPT) $(INCDIR) $(shell pkg-config --cflags opencv)
#-c -g -Wall

#LINK   = -lm -L/usr/local/lib -lcxcore -lcv -lhighgui -lcvaux -lml 
#LINK = -lm -L/usr/local/lib -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_video -lopencv_features2d -lopencv_ml -lopencv_highgui -lopencv_objdetect -lopencv_contrib -lopencv_legacy
LINK = $(shell pkg-config --libs opencv)

.cpp.o:
	$(CPP) $(CFLAGS) -c $< -o $@

all: ground_plane_segmentation

ground_plane_segmentation: main.cpp init_structures.cpp img_proc_fcns.cpp capture.cpp ms_overwrite_safe_buffer.cpp segment-image.h segment-graph.h disjoint-set.h Net.h ms_communications_loop.h
	$(CPP) $(CFLAGS) -o ground_plane_segmentation main.cpp init_structures.cpp img_proc_fcns.cpp capture.cpp ms_overwrite_safe_buffer.cpp ms_communications_loop.h $(LINK)

clean:
	rm -f ground_plane_segmentation *.o

clean-all: clean
	rm -f *~

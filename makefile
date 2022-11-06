CC = g++
CFLAGS = -g -Wall

SRCS = detect_markers_with_threading.cpp
PROG = detect_markers_with_threading

OPENCV = `pkg-config opencv --cflags --libs` -lpthread
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)



#g++ -ggdb facedetect.cpp -o facedetect `pkg-config --cflags --libs opencv`

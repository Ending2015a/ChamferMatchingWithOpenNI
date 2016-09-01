CC = g++
CFLAGS = -g -Wall -I/usr/include -I/usr/include/openni2/ -L/usr/local/lib/
opencvlibs = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching
opennilib = -lOpenNI2

chamfer: Source.o
	${CC} ${CFLAGS} Source.o -o $@ ${opencvlibs} ${opennilibs}
Source.o: Source.cpp
	${CC} ${CFLAGS} -c $< -o $@ ${opencvlibs} ${opennilibs}
clean:
	@rm -rf *.o
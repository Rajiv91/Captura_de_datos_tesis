INCLUDE=./include/
OPENCV_LIBS=`pkg-config --libs opencv`
CFLAGS = `pkg-config opencv --cflags`
CLIBS =  `pkg-config opencv --libs`
INCLUDEDIR=-I$(INCLUDE) -I/usr/local/include/opencv -I/usr/local/include -I. $(CFLAGS)
LIBDIRS=-L/usr/local/lib/
LIBS =  -lm $(OPENCV_LIBS) -lrt -lm -ldl -pthread

all:test

test: objs/grid.o
	g++ -o $@ $^ $(CFLAGS) $(CLIBS) $(LIBDIRS) $(LIBS)

#we define a rule that applies to all files ending in the .o suffix. The rule says that the .o file depends upon the .c version of the file
objs/%.o: %.cpp #$(INCLUDE)Etiquetacion.h $(INCLUDE)Blob.h
	g++ -fPIE -c -o $@ $< $(CLIBS) $(INCLUDEDIR) #$(LIBDIRS) $(LIBS)
	#g++ -I$(INCLUDE) -c -o objs/main.o main.cpp #Igual funciona
#-I$(INCLUDE)
clean:
	rm ./objs/* test

#the -o $@ says to put the output of the compilation in the file named on the left side of the :
 #the special macros $@ and $^, which are the left and right sides of the :
 #$< The name of the first prerequisite.

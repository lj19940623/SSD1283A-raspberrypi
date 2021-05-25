.PHONY: all clean
INCLUDES = -I./
LIBS = -lwiringPi
objects = test.o SSD1283A.o
all: exec
exec : $(objects)
	g++  $(objects) $(LIBS) -o exec
clean:
	rm $(objects) exec

test: test.o
	cc test.o -o test
test.o: test.cpp
	cc -c test.cpp
	
SSD1283A.o:  SSD1283A.cpp
	cc  -fpermissive -c SSD1283A.cpp

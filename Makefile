ifeq ($(shell uname), Linux)
   LINUX_BT = 1 # remove or comment this line if compiling in Linux without BlueZ development files
endif

ifdef LINUX_BT
   CFLAGS = -DHASBLUETOOTH
   LIBS = -lbluetooth
endif 

all: test

test: bitalino.o test.o
	g++ $^ $(LIBS) -o $@

bitalino.o: bitalino.cpp
	g++ $(CFLAGS) -c $< -o $@

test.o: test.cpp
	g++ -std=c++0x -c $< -o $@
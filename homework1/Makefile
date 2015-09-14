CC=g++
CCFLAGS=-std=c++11
LIBS=

all: p1 p2 p3 p4

p1: p1_test.cpp
	$(CC) $(CCFLAGS) $(LIBS) p1_test.cpp -o p1

p2:
	$(CC) $(CCFLAGS) $(LIBS) p2_test.cpp -o p2

p3: p3_test.cpp
	$(CC) $(CCFLAGS) $(LIBS) p3_test.cpp -o p3

p4: p4_test.cpp
	$(CC) $(CCFLAGS) $(LIBS) p4_test.cpp -o p4

clean:
	rm *.o p1 p2 p3 p4

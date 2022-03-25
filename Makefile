CXX = g++
CC = gcc
CPPOBJS = src/main.o src/example.o
COBJS =
EDCXXFLAGS = -I ./ -I ./include/ -I ./GLEE2023/src/ -Wall -pthread $(CXXFLAGS)
EDCFLAGS = $(CFLAGS)
EDLDFLAGS := -lpthread -lm $(LDFLAGS)
TARGET = example.out

all: $(COBJS) $(CPPOBJS)
	$(CXX) $(EDCXXFLAGS) $(COBJS) $(CPPOBJS) -o $(TARGET) $(EDLDFLAGS)
	sudo ./$(TARGET)

%.o: %.cpp
	$(CXX) $(EDCXXFLAGS) -o $@ -c $<

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	$(RM) *.out
	$(RM) *.o
	$(RM) src/*.o

.PHONY: spotless

spotless:
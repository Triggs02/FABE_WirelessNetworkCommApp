# Makefile used to generate the executable to run the ECE Capstone team's
# wireless communication system.

CC=g++
CFLAGS=-c -Wall -Iinclude
LDFLAGS=-lbcm2835 -lSSD1306_OLED_RPI
CPP_SOURCES=$(wildcard src/*.cpp)
C_SOURCES=$(wildcard src/*.c)
CPP_OBJECTS=$(addprefix obj/,$(notdir $(CPP_SOURCES:.cpp=.o)))
C_OBJECTS=$(addprefix obj/,$(notdir $(C_SOURCES:.c=.o)))
EXECUTABLE=bin/wirelessNetworkApp

all: obj build $(EXECUTABLE)

obj:
	mkdir obj

build:
	mkdir bin

$(EXECUTABLE): $(CPP_OBJECTS) $(C_OBJECTS)
	$(CC) $(CPP_OBJECTS) $(C_OBJECTS) -o $@ $(LDFLAGS)

obj/%.o: src/%.cpp
	$(CC) $(CFLAGS) $< -o $@

obj/%.o: src/%.c
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf obj bin

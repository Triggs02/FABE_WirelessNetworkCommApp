# Makefile used to generate the executable to run the ECE Capstone team's
# wireless communication system.

CC=g++
CFLAGS=-c -Wall
LDFLAGS=-lbcm2835 -lSSD1306_OLED_RPI
INC_DIR=include
SRC_DIR=src
OBJ_DIR=obj
EXECUTABLE=wirelessNetworkApp

# Get all source files
CPP_SOURCES=$(wildcard $(SRC_DIR)/*.cpp)
C_SOURCES=$(wildcard $(SRC_DIR)/*.c)

# Generate object file names
CPP_OBJECTS=$(addprefix $(OBJ_DIR)/,$(notdir $(CPP_SOURCES:.cpp=.o)))
C_OBJECTS=$(addprefix $(OBJ_DIR)/,$(notdir $(C_SOURCES:.c=.o)))

# Include directories
INC=-I$(INC_DIR)

all: obj $(EXECUTABLE)

obj:
	mkdir obj

$(EXECUTABLE): $(CPP_OBJECTS) $(C_OBJECTS)
	$(CC) $^ -o $(EXECUTABLE) $(LDFLAGS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CFLAGS) $(INC) $< -o $@

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) $(CFLAGS) $(INC) $< -o $@

clean:
	rm -rf obj $(EXECUTABLE)





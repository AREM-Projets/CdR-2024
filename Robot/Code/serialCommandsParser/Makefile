# Compiler, flags, executable_name
CC = g++
CFLAGS = -g -Wall -Wextra # -g : debug flag
EXECUTABLE = app

# Directories
SRC_DIR = src
BUILD_DIR = build
INCLUDE_DIR = lib

#---------------------------------------------------------------#
#--/ It is recommended to NOT TOUCH anything below that line /--#
#---------------------------------------------------------------#

# Source and object files
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
OBJ_FILES = $(patsubst $(SRC_DIR)/%.cpp,$(BUILD_DIR)/%.o,$(SRC_FILES))

# Automatically find library directories
LIBRARY_DIRS = $(wildcard $(INCLUDE_DIR)/*/) $(INCLUDE_DIR)/

# Include directories for libraries
LIBRARY_INCLUDES = $(foreach dir, $(LIBRARY_DIRS), -I$(dir))

# Library source files
LIBRARY_SRC_FILES = $(wildcard $(addsuffix *.cpp, $(LIBRARY_DIRS)))
LIBRARY_OBJ_FILES = $(patsubst %.cpp, $(BUILD_DIR)/%.o, $(notdir $(LIBRARY_SRC_FILES)))

# Default target
all: $(EXECUTABLE)

# Compile main source files into object files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CC) $(CFLAGS) $(LIBRARY_INCLUDES) -c $< -o $@

# Compile library source files into object files
$(BUILD_DIR)/%.o: $(INCLUDE_DIR)/*/%.cpp # Libraries in separate folders
	$(CC) $(CFLAGS) $(LIBRARY_INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: $(INCLUDE_DIR)/%.cpp # libraries directly in include/
	$(CC) $(CFLAGS) $(LIBRARY_INCLUDES) -c $< -o $@

# Link object files to create the executable
$(EXECUTABLE): $(OBJ_FILES) $(LIBRARY_OBJ_FILES)
	$(CC) $(CFLAGS) $^ -o $@

# Clean build artifacts
clean:
	rm -f $(BUILD_DIR)/*.o
	rm -f $(EXECUTABLE)

run:
	make $(EXECUTABLE)
	./$(EXECUTABLE)


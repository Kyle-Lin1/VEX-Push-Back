# Makefile for EKF Unit Tests
# This builds and runs isolated unit tests for the EKF implementation
# Cross-platform support for Windows, Linux, and macOS

# Detect OS
ifeq ($(OS),Windows_NT)
    DETECTED_OS := Windows
else
    DETECTED_OS := $(shell uname -s)
endif

# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I../include -I.

# Platform-specific settings
ifeq ($(DETECTED_OS),Windows)
    TEST_BIN = test_ekf.exe
    RM = del /Q
    LDFLAGS =
    RUN_TEST = test_ekf.exe
else
    TEST_BIN = test_ekf
    RM = rm -f
    LDFLAGS = -lm
    RUN_TEST = ./test_ekf
endif

# Source files
EKF_SRC = ../src/subsystemFunctions/ekf.cpp
TEST_SRC = test_ekf.cpp

# Build target
all: $(TEST_BIN)

$(TEST_BIN): $(TEST_SRC) $(EKF_SRC) test_framework.hpp
	@echo "Building EKF unit tests for $(DETECTED_OS)..."
	$(CXX) $(CXXFLAGS) $(TEST_SRC) $(EKF_SRC) -o $(TEST_BIN) $(LDFLAGS)
	@echo "Build complete!"

# Run tests
test: $(TEST_BIN)
	@echo ""
	@echo "Running EKF unit tests..."
	@echo ""
	$(RUN_TEST)

# Run tests with verbose output
test-verbose: $(TEST_BIN)
	@echo ""
	@echo "Running EKF unit tests (verbose)..."
	@echo ""
	$(RUN_TEST) -v

# Clean build artifacts
clean:
	@echo "Cleaning test artifacts..."
	-$(RM) $(TEST_BIN)
	@echo "Clean complete!"

# Rebuild everything
rebuild: clean all

# Run tests and clean up
test-clean: test clean

.PHONY: all test test-verbose clean rebuild test-clean


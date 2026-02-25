# Makefile for EKF Unit Tests
# This builds and runs isolated unit tests for the EKF implementation

CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -I../include -I.
LDFLAGS = -lm

# Source files
EKF_SRC = ../src/subsystemFunctions/ekf.cpp
TEST_SRC = test_ekf.cpp

# Output binary
TEST_BIN = test_ekf

# Build target
all: $(TEST_BIN)

$(TEST_BIN): $(TEST_SRC) $(EKF_SRC) test_framework.hpp
	@echo "Building EKF unit tests..."
	$(CXX) $(CXXFLAGS) $(TEST_SRC) $(EKF_SRC) -o $(TEST_BIN) $(LDFLAGS)
	@echo "Build complete!"

# Run tests
test: $(TEST_BIN)
	@echo ""
	@echo "Running EKF unit tests..."
	@echo ""
	./$(TEST_BIN)

# Run tests with verbose output
test-verbose: $(TEST_BIN)
	@echo ""
	@echo "Running EKF unit tests (verbose)..."
	@echo ""
	./$(TEST_BIN) -v

# Clean build artifacts
clean:
	@echo "Cleaning test artifacts..."
	rm -f $(TEST_BIN)
	@echo "Clean complete!"

# Rebuild everything
rebuild: clean all

# Run tests and clean up
test-clean: test clean

.PHONY: all test test-verbose clean rebuild test-clean


# EKF Unit Tests

This directory contains comprehensive unit tests for the Extended Kalman Filter (EKF) implementation used in the VEX robot odometry system.

## Overview

The tests are designed to run **independently** of the VEX PROS environment, allowing you to verify the EKF implementation on your development machine without needing to deploy to the robot.

## Test Coverage

The test suite includes the following test cases:

1. **Initialization Test** - Verifies proper initialization of state variables
2. **Forward Movement** - Tests pure forward motion at 0 degrees
3. **Strafe Movement** - Tests pure lateral (strafe) motion
4. **Forward at 90 Degrees** - Tests coordinate transformation at different headings
5. **IMU Correction** - Verifies IMU sensor fusion and heading correction
6. **Combined Movement** - Tests simultaneous forward and strafe motion
7. **setPose Functionality** - Tests position reset capability
8. **Angle Wrapping** - Verifies proper handling of angle discontinuities at ±180°
9. **Sequential Movements** - Tests multiple consecutive movements
10. **Noise Parameter Tuning** - Verifies that noise parameters affect filter behavior
11. **Backward Movement** - Tests negative velocity inputs
12. **Full Rotation** - Tests 360-degree rotation handling

## Building and Running Tests

### Prerequisites

- C++ compiler with C++17 support (g++, clang++)
- Make

### Quick Start

```bash
# Navigate to the tests directory
cd tests

# Build and run all tests
make test

# Or build first, then run
make
./test_ekf
```

### Available Make Targets

- `make` or `make all` - Build the test executable
- `make test` - Build and run all tests
- `make clean` - Remove build artifacts
- `make rebuild` - Clean and rebuild
- `make test-clean` - Run tests and clean up afterward

## Understanding Test Output

The test framework provides colored output for easy reading:

- **CYAN** - Test name and start
- **GREEN** - Test passed (✓)
- **RED** - Test failed (✗)
- **YELLOW** - Assertion details (expected vs actual values)

### Example Output

```
========================================
  EKF Unit Tests
========================================

[TEST] EKF Initialization...
  ✓ PASS

[TEST] EKF Forward Movement...
  ✓ PASS

[TEST] EKF IMU Heading Correction...
  ✗ FAIL: Theta should be between 0 and 10
    Expected: true
    Got: false

========================================
  Test Summary
========================================
Total tests: 12
Passed: 11
Failed: 1
========================================
Some tests failed! ✗
```

## Interpreting Results

### All Tests Pass ✓
If all tests pass, your EKF implementation is working correctly for the tested scenarios.

### Some Tests Fail ✗
If tests fail, check:
1. **Jacobian Matrix** - Ensure the linearization is correct
2. **Coordinate Transformations** - Verify robot-to-global frame conversions
3. **Angle Wrapping** - Check handling of ±180° discontinuities
4. **Noise Parameters** - Ensure Q and R matrices are properly initialized

## Tuning the EKF

The EKF has several tunable parameters that affect its behavior:

### Process Noise (`q_pos`, `q_theta`)
- **Higher values** → Filter reacts faster to changes, but may be noisier
- **Lower values** → Filter is smoother, but may lag behind actual motion
- Default: `q_pos = 0.01`, `q_theta = 0.005`

### Measurement Noise (`r_imu`)
- **Higher values** → Filter trusts IMU less, relies more on encoders
- **Lower values** → Filter trusts IMU more for heading correction
- Default: `r_imu = 0.02`

You can modify these in the EKF class and re-run tests to see the effects.

## Adding New Tests

To add a new test:

1. Create a new test function in `test_ekf.cpp`:
```cpp
void test_my_new_feature() {
    TEST_START("My New Feature");
    
    EKF ekf;
    ekf.initialize(0.0, 0.0, 0.0);
    
    // Your test code here
    ASSERT_APPROX_EQUAL(ekf.getX(), expected_value, 0.01, "Description");
    
    TEST_PASS();
}
```

2. Add the function call to `main()`:
```cpp
int main() {
    // ... existing tests ...
    test_my_new_feature();
    
    PRINT_TEST_SUMMARY();
    return TEST_RETURN_CODE();
}
```

3. Rebuild and run: `make test`

## Test Framework Macros

- `TEST_START(name)` - Begin a test
- `TEST_PASS()` - Mark test as passed
- `TEST_FAIL(message)` - Mark test as failed with message
- `ASSERT_TRUE(condition, message)` - Assert condition is true
- `ASSERT_FALSE(condition, message)` - Assert condition is false
- `ASSERT_EQUAL(actual, expected, message)` - Assert exact equality
- `ASSERT_APPROX_EQUAL(actual, expected, tolerance, message)` - Assert approximate equality
- `PRINT_TEST_SUMMARY()` - Print final test results

## Troubleshooting

### Compilation Errors
- Ensure you're in the `tests/` directory
- Check that `../include/subsystemHeaders/ekf.hpp` exists
- Verify C++17 support: `g++ --version`

### Test Failures
- Review the specific assertion that failed
- Check expected vs actual values
- Consider if the tolerance is appropriate for your use case
- Verify the test assumptions match your implementation

## Integration with Robot Code

These tests verify the EKF in isolation. To verify integration with the robot:

1. Deploy code to the robot
2. Monitor the EKF output via PROS terminal or LCD
3. Compare EKF pose estimates with LemLib's odometry
4. Tune noise parameters based on real-world performance

## License

Part of the VEX-Push-Back robot project.


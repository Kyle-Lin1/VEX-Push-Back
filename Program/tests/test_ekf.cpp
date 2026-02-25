#include "test_framework.hpp"
#include "../include/subsystemHeaders/ekf.hpp"
#include <cmath>
#include <iostream>

// Helper function to compare doubles with tolerance
bool approxEqual(double a, double b, double tolerance = 1e-6) {
    return std::abs(a - b) < tolerance;
}

// Test 1: Initialization
void test_initialization() {
    TEST_START("EKF Initialization");
    
    EKF ekf;
    ekf.initialize(10.0, 20.0, 45.0);
    
    ASSERT_APPROX_EQUAL(ekf.getX(), 10.0, 1e-6, "X should be initialized to 10.0");
    ASSERT_APPROX_EQUAL(ekf.getY(), 20.0, 1e-6, "Y should be initialized to 20.0");
    ASSERT_APPROX_EQUAL(ekf.getTheta(), 45.0, 1e-6, "Theta should be initialized to 45.0 degrees");
    
    TEST_PASS();
}

// Test 2: Pure forward movement
void test_forward_movement() {
    TEST_START("EKF Forward Movement");
    
    EKF ekf;
    ekf.initialize(0.0, 0.0, 0.0);
    
    // Move forward 10 inches at 0 degrees heading
    ekf.update(10.0, 0.0, 0.0);
    
    ASSERT_APPROX_EQUAL(ekf.getX(), 10.0, 0.01, "X should increase by 10");
    ASSERT_APPROX_EQUAL(ekf.getY(), 0.0, 0.01, "Y should remain 0");
    ASSERT_APPROX_EQUAL(ekf.getTheta(), 0.0, 0.01, "Theta should remain 0");
    
    TEST_PASS();
}

// Test 3: Pure strafe movement
void test_strafe_movement() {
    TEST_START("EKF Strafe Movement");

    EKF ekf;
    ekf.initialize(0.0, 0.0, 0.0);

    // Strafe right 5 inches at 0 degrees heading
    // At 0 degrees: horizontal_delta positive means strafe right (positive Y in global frame)
    ekf.update(0.0, 5.0, 0.0);

    ASSERT_APPROX_EQUAL(ekf.getX(), 0.0, 0.01, "X should remain 0");
    ASSERT_APPROX_EQUAL(ekf.getY(), 5.0, 0.01, "Y should increase by 5 (right strafe)");
    ASSERT_APPROX_EQUAL(ekf.getTheta(), 0.0, 0.01, "Theta should remain 0");

    TEST_PASS();
}

// Test 4: Forward movement at 90 degrees
void test_forward_at_90_degrees() {
    TEST_START("EKF Forward at 90 Degrees");
    
    EKF ekf;
    ekf.initialize(0.0, 0.0, 90.0);
    
    // Move forward 10 inches at 90 degrees heading
    ekf.update(10.0, 0.0, 90.0);
    
    ASSERT_APPROX_EQUAL(ekf.getX(), 0.0, 0.01, "X should remain near 0");
    ASSERT_APPROX_EQUAL(ekf.getY(), 10.0, 0.01, "Y should increase by 10");
    ASSERT_APPROX_EQUAL(ekf.getTheta(), 90.0, 0.01, "Theta should remain 90");
    
    TEST_PASS();
}

// Test 5: IMU heading correction
void test_imu_correction() {
    TEST_START("EKF IMU Heading Correction");
    
    EKF ekf;
    ekf.initialize(0.0, 0.0, 0.0);
    
    // Simulate encoder drift vs IMU
    // Encoders say we're still at 0, but IMU says we're at 10 degrees
    ekf.update(0.0, 0.0, 10.0);
    
    // Theta should move toward IMU reading (not exactly 10 due to Kalman filtering)
    double theta = ekf.getTheta();
    ASSERT_TRUE(theta > 0.0 && theta <= 10.0, "Theta should be between 0 and 10");
    
    TEST_PASS();
}

// Test 6: Combined movement
void test_combined_movement() {
    TEST_START("EKF Combined Forward and Strafe");
    
    EKF ekf;
    ekf.initialize(0.0, 0.0, 45.0);
    
    // Move forward and strafe at 45 degrees
    ekf.update(10.0, 5.0, 45.0);
    
    // At 45 degrees:
    // forward component: 10*cos(45) - 5*sin(45) ≈ 7.07 - 3.54 = 3.53
    // strafe component: 10*sin(45) + 5*cos(45) ≈ 7.07 + 3.54 = 10.61
    
    ASSERT_APPROX_EQUAL(ekf.getX(), 3.54, 0.1, "X should be ~3.54");
    ASSERT_APPROX_EQUAL(ekf.getY(), 10.61, 0.1, "Y should be ~10.61");
    
    TEST_PASS();
}

// Test 7: setPose functionality
void test_set_pose() {
    TEST_START("EKF setPose");
    
    EKF ekf;
    ekf.initialize(0.0, 0.0, 0.0);
    
    // Move somewhere
    ekf.update(10.0, 5.0, 15.0);
    
    // Reset pose
    ekf.setPose(100.0, 200.0, 90.0);
    
    ASSERT_APPROX_EQUAL(ekf.getX(), 100.0, 1e-6, "X should be reset to 100");
    ASSERT_APPROX_EQUAL(ekf.getY(), 200.0, 1e-6, "Y should be reset to 200");
    ASSERT_APPROX_EQUAL(ekf.getTheta(), 90.0, 1e-6, "Theta should be reset to 90");
    
    TEST_PASS();
}

// Test 8: Angle wrapping
void test_angle_wrapping() {
    TEST_START("EKF Angle Wrapping");

    EKF ekf;
    ekf.initialize(0.0, 0.0, 170.0);

    // IMU reports 190 degrees (should wrap properly)
    ekf.update(0.0, 0.0, 190.0);

    double theta = ekf.getTheta();
    // Should handle the wrap from 170 to 190 (crossing 180/-180 boundary)
    ASSERT_TRUE(theta > 170.0 || theta < -170.0, "Theta should handle wrapping");

    TEST_PASS();
}

// Test 9: Sequential movements
void test_sequential_movements() {
    TEST_START("EKF Sequential Movements");

    EKF ekf;
    ekf.initialize(0.0, 0.0, 0.0);

    // Move forward 10 inches at 0 degrees
    ekf.update(10.0, 0.0, 0.0);
    double x_after_first = ekf.getX();
    ASSERT_APPROX_EQUAL(x_after_first, 10.0, 0.01, "After first move, X should be 10");

    // Turn 90 degrees (IMU only, no encoder movement)
    ekf.update(0.0, 0.0, 90.0);
    double x_after_turn = ekf.getX();
    double y_after_turn = ekf.getY();
    // Position might change slightly due to Kalman correction, but should be close
    ASSERT_APPROX_EQUAL(x_after_turn, x_after_first, 0.5, "X should be close to previous value");

    // Move forward 10 more inches (now heading is close to 90 degrees)
    ekf.update(10.0, 0.0, 90.0);
    double theta = ekf.getTheta();
    double y_final = ekf.getY();

    // At ~90 degrees, forward motion should primarily increase Y
    // The Y change should be approximately 10 inches from the previous Y position
    double y_delta = y_final - y_after_turn;
    ASSERT_APPROX_EQUAL(y_delta, 10.0, 1.0, "Y should increase by ~10 from previous position");

    // The Kalman filter gradually converges to the IMU reading
    // After only 2 updates with IMU at 90, theta might not be exactly at 90 yet
    // This is expected behavior - the filter doesn't jump instantly to measurements
    ASSERT_TRUE(theta > 50.0 && theta < 100.0, "Theta should be converging toward 90 degrees");

    TEST_PASS();
}

// Test 10: Noise parameter tuning
void test_noise_parameters() {
    TEST_START("EKF Noise Parameter Tuning");

    EKF ekf;
    ekf.initialize(0.0, 0.0, 0.0);

    // Test with default parameters
    ekf.update(0.0, 0.0, 10.0);
    double theta1 = ekf.getTheta();

    // Reset and test with high IMU noise (should trust IMU less)
    ekf.initialize(0.0, 0.0, 0.0);
    ekf.r_imu = 10.0; // Very high noise
    ekf.update(0.0, 0.0, 10.0);
    double theta2 = ekf.getTheta();

    // With higher IMU noise, the filter should trust the IMU less
    ASSERT_TRUE(theta2 < theta1, "Higher IMU noise should result in less correction");

    TEST_PASS();
}

// Test 11: Backward movement
void test_backward_movement() {
    TEST_START("EKF Backward Movement");

    EKF ekf;
    ekf.initialize(0.0, 0.0, 0.0);

    // Move backward 10 inches (negative vertical delta)
    ekf.update(-10.0, 0.0, 0.0);

    ASSERT_APPROX_EQUAL(ekf.getX(), -10.0, 0.01, "X should decrease by 10");
    ASSERT_APPROX_EQUAL(ekf.getY(), 0.0, 0.01, "Y should remain 0");

    TEST_PASS();
}

// Test 12: Gradual rotation convergence
void test_full_rotation() {
    TEST_START("EKF Gradual Rotation Convergence");

    EKF ekf;
    ekf.initialize(0.0, 0.0, 0.0);

    // Simulate gradual rotation by repeatedly feeding the same IMU angle
    // This tests that the filter converges to the IMU reading over time
    double target_angle = 90.0;

    // Feed the same angle multiple times to allow convergence
    for (int i = 0; i < 20; i++) {
        ekf.update(0.0, 0.0, target_angle);
    }

    // After many updates with the same IMU reading, theta should converge close to it
    double final_theta = ekf.getTheta();
    ASSERT_APPROX_EQUAL(final_theta, target_angle, 5.0,
                        "After repeated updates, theta should converge to IMU reading");

    // Position should remain near origin (no encoder movement)
    ASSERT_APPROX_EQUAL(ekf.getX(), 0.0, 0.5, "X should remain near 0");
    ASSERT_APPROX_EQUAL(ekf.getY(), 0.0, 0.5, "Y should remain near 0");

    TEST_PASS();
}

// Main test runner
int main() {
    std::cout << "\n========================================\n";
    std::cout << "  EKF Unit Tests\n";
    std::cout << "========================================\n\n";

    test_initialization();
    test_forward_movement();
    test_strafe_movement();
    test_forward_at_90_degrees();
    test_imu_correction();
    test_combined_movement();
    test_set_pose();
    test_angle_wrapping();
    test_sequential_movements();
    test_noise_parameters();
    test_backward_movement();
    test_full_rotation();

    PRINT_TEST_SUMMARY();

    return TEST_RETURN_CODE();
}


#pragma once
#include <array>

/**
 * Extended Kalman Filter (EKF) for robot odometry
 * Tracks x, y position and heading (theta).
 *
 * Uses vertical/horizontal tracking wheel encoders and IMU for heading correction.
 * Properly handles cross-correlations between position and heading errors.
 */
class EKF {
public:
    // Default constructor
    EKF();

    // Initialize the filter with starting pose
    void initialize(double x_0, double y_0, double theta_0);

    // Reset pose (call when using lemlib's setPose())
    void setPose(double x_0, double y_0, double theta_0);

    // Main update using vertical/horizontal encoder deltas and IMU heading
    // vertical_delta: forward/backward movement in inches
    // horizontal_delta: left/right strafe movement in inches
    // imu_heading: IMU heading in degrees
    void update(double vertical_delta, double horizontal_delta, double imu_heading);

    // Accessors for robot pose
    double getX() const;
    double getY() const;
    double getTheta() const; // returns theta in degrees

    // Tunable noise parameters (public for easy tuning)
    // Process noise - increase if filter reacts too slowly
    double q_pos;      // position process noise
    double q_theta;    // heading process noise

    // Measurement noise - increase if IMU is noisy
    double r_imu;      // IMU measurement noise

private:
    // Prediction step using encoder deltas
    void predict(double vertical_delta, double horizontal_delta);

    // Update step using IMU heading measurement
    void updateIMU(double imu_heading_deg);

    // Robot state (x, y in inches, theta in radians internally)
    double x, y, theta;

    // Covariance matrix (3x3)
    std::array<std::array<double, 3>, 3> P;

    // Process noise covariance matrix (3x3)
    std::array<std::array<double, 3>, 3> Q;

    // Measurement noise (scalar for IMU)
    double R;
};

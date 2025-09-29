#pragma once
#include <array>

/**
 * Simple Extended Kalman Filter (EKF) for robot odometry
 * Tracks x, y position and heading (theta).
 *
 * Uses left/right wheel encoders for distance and IMU for heading correction.
 */
class EKF {
public:
    // constructor (wheel radius, wheel base, timestep)
    EKF(double wheelRadius, double wheelBase, double dtIn);

    // main update (feed in encoder distances and imu heading)
    void update(double leftDist, double rightDist, double imuHeading);

    // accessors for robot pose
    double getX() const;
    double getY() const;
    double getTheta() const;

private:
    // prediction and correction steps
    void predict(double dl, double dr);
    void updateIMU(double imuHeading);

    // robot state
    double x, y, theta;
    double r, b, dt;

    // covariance matrix
    std::array<std::array<double, 3>, 3> P;

    // process noise
    std::array<std::array<double, 3>, 3> Q;

    // measurement noise (IMU)
    double R;
};

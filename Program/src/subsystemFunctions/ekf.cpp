#include "subsystemHeaders/ekf.hpp"
#include <cmath>

// Define M_PI if not already defined (needed for some Windows compilers)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper function to wrap angle to [-pi, pi]
static double wrapAngle(double rad) {
    while (rad > M_PI) rad -= 2.0 * M_PI;
    while (rad < -M_PI) rad += 2.0 * M_PI;
    return rad;
}

// Default constructor
EKF::EKF() {
    // Initialize state to origin
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    // Start with small uncertainty
    P = {{{0.1, 0, 0},
          {0, 0.1, 0},
          {0, 0, 0.05}}};

    // Default tunable parameters
    q_pos = 0.01;      // position process noise
    q_theta = 0.005;   // heading process noise
    r_imu = 0.02;      // IMU measurement noise

    // Initialize process noise matrix
    Q = {{{q_pos, 0, 0},
          {0, q_pos, 0},
          {0, 0, q_theta}}};

    // Initialize measurement noise
    R = r_imu;
}

// Initialize with starting pose
void EKF::initialize(double x_0, double y_0, double theta_0) {
    x = x_0;
    y = y_0;
    theta = theta_0 * (M_PI / 180.0); // convert degrees to radians

    // Reset covariance to small uncertainty
    P = {{{0.1, 0, 0},
          {0, 0.1, 0},
          {0, 0, 0.05}}};
}

// Reset pose (for when lemlib's setPose() is used)
void EKF::setPose(double x_0, double y_0, double theta_0) {
    x = x_0;
    y = y_0;
    theta = theta_0 * (M_PI / 180.0); // convert degrees to radians

    // Reset covariance for confidence in known position
    P = {{{0.1, 0, 0},
          {0, 0.1, 0},
          {0, 0, 0.05}}};
}
// Predict step: use vertical and horizontal encoder deltas
void EKF::predict(double vertical_delta, double horizontal_delta) {
    // Update process noise matrix with current tunable parameters
    Q[0][0] = q_pos;
    Q[1][1] = q_pos;
    Q[2][2] = q_theta;

    // Motion model: robot-centric to global frame transformation
    // vertical_delta: forward/backward in robot frame
    // horizontal_delta: left/right strafe in robot frame
    double dx = vertical_delta * cos(theta) - horizontal_delta * sin(theta);
    double dy = vertical_delta * sin(theta) + horizontal_delta * cos(theta);

    // Update state
    x += dx;
    y += dy;
    // theta doesn't change from encoders alone (only from IMU)

    // Compute Jacobian of motion model with respect to state
    // f(x,y,theta) = [x + v*cos(theta) - h*sin(theta),
    //                 y + v*sin(theta) + h*cos(theta),
    //                 theta]
    // where v = vertical_delta, h = horizontal_delta
    //
    // Jacobian Fx = df/d[x,y,theta] =
    // [1,  0,  -v*sin(theta) - h*cos(theta)]
    // [0,  1,   v*cos(theta) - h*sin(theta)]
    // [0,  0,   1                           ]

    double Fx[3][3] = {
        {1.0, 0.0, -vertical_delta * sin(theta) - horizontal_delta * cos(theta)},
        {0.0, 1.0,  vertical_delta * cos(theta) - horizontal_delta * sin(theta)},
        {0.0, 0.0,  1.0}
    };

    // Update covariance: P = Fx * P * Fx^T + Q
    // Step 1: Compute FP = Fx * P
    double FP[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                FP[i][j] += Fx[i][k] * P[k][j];
            }
        }
    }

    // Step 2: Compute newP = FP * Fx^T + Q
    std::array<std::array<double, 3>, 3> newP = {{{0,0,0},{0,0,0},{0,0,0}}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                newP[i][j] += FP[i][k] * Fx[j][k]; // Fx[j][k] is Fx^T[k][j]
            }
            newP[i][j] += Q[i][j];
        }
    }
    P = newP;
}
// Update step: use IMU heading measurement
void EKF::updateIMU(double imu_heading_deg) {
    // Update measurement noise with current tunable parameter
    R = r_imu;

    // Convert IMU heading from degrees to radians
    double z = imu_heading_deg * (M_PI / 180.0);

    // Compute innovation (measurement residual)
    double yk = z - theta;
    yk = wrapAngle(yk); // wrap to [-pi, pi]

    // Innovation covariance: S = H * P * H^T + R
    // where H = [0, 0, 1] (we're measuring theta only)
    // S = P[2][2] + R
    double S = P[2][2] + R;

    // Kalman gain: K = P * H^T * S^-1
    // K = [P[0][2], P[1][2], P[2][2]]^T / S
    double K[3] = { P[0][2] / S, P[1][2] / S, P[2][2] / S };

    // Update state: x = x + K * yk
    x += K[0] * yk;
    y += K[1] * yk;
    theta += K[2] * yk;

    // Wrap theta to [-pi, pi]
    theta = wrapAngle(theta);

    // Update covariance: P = (I - K*H) * P
    // H = [0, 0, 1], so K*H is a matrix where only the 3rd column is non-zero
    double KH[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            KH[i][j] = K[i] * (j == 2 ? 1.0 : 0.0); // H = [0 0 1]
        }
    }

    // Compute I - K*H
    double I_KH[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
        }
    }

    // Compute newP = (I - K*H) * P
    std::array<std::array<double, 3>, 3> newP = {{{0,0,0},{0,0,0},{0,0,0}}};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                newP[i][j] += I_KH[i][k] * P[k][j];
            }
        }
    }
    P = newP;
}

// Main update function
void EKF::update(double vertical_delta, double horizontal_delta, double imu_heading) {
    predict(vertical_delta, horizontal_delta);
    updateIMU(imu_heading);
}

// Getters
double EKF::getX() const {
    return x;
}

double EKF::getY() const {
    return y;
}

double EKF::getTheta() const {
    // Convert from radians to degrees for output
    return theta * (180.0 / M_PI);
}
#include "subsystemHeaders/ekf.hpp"
#include <cmath>

// constructors
EKF::EKF(double wheelRadius, double wheelBase, double dtIn) {
    x = 0;
    y = 0;
    theta = 0;
    r = wheelRadius;
    b = wheelBase;
    dt = dtIn;

    // start with small uncertainty
    P = {{{0.1, 0, 0},
          {0, 0.1, 0},
          {0, 0, 0.05}}};

    // process noise (tuning values)
    Q = {{{0.01, 0, 0},
          {0, 0.01, 0},
          {0, 0, 0.005}}};

    // imu noise (tune this too)
    R = 0.02;
}



// predict step: use encoder distances
void EKF::predict(double dl, double dr) {
    double dtheta = (dr - dl) / b;
    double d = (dl + dr) / 2.0;

    // predicted position change
    double dx = d * cos(theta + dtheta / 2.0);
    double dy = d * sin(theta + dtheta / 2.0);

    // update state
    x += dx;
    y += dy;
    theta += dtheta;

    // wrap angle between -pi and pi
    if (theta > M_PI) theta -= 2 * M_PI;
    if (theta < -M_PI) theta += 2 * M_PI;

    // Jacobian for linearization
    double Fx[3][3] = {
        {1, 0, -dy},
        {0, 1,  dx},
        {0, 0,  1}
    };

// update covariance: P = Fx * P * Fx^T + Q
double FP[3][3] = {0};
for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
            FP[i][j] += Fx[i][k] * P[k][j];
        }
    }
}

std::array<std::array<double, 3>, 3> newP = {{{0,0,0},{0,0,0},{0,0,0}}};
for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
        for (int k = 0; k < 3; k++) {
            newP[i][j] += FP[i][k] * Fx[j][k]; // Fx^T
        }
        newP[i][j] += Q[i][j];
    }
}
P = newP;
}

// update step: use imu heading
void EKF::updateIMU(double imuHeading) {
    // measurement z = imu heading
    double z = imuHeading;

    // innovation
    double yk = z - theta;
    if (yk > M_PI) yk -= 2 * M_PI;
    if (yk < -M_PI) yk += 2 * M_PI;

    // S = P[2][2] + R
    double S = P[2][2] + R;

    // Kalman gain
    double K[3] = { P[0][2] / S, P[1][2] / S, P[2][2] / S };

    // update state
    x += K[0] * yk;
    y += K[1] * yk;
    theta += K[2] * yk;

    // wrap theta again
    if (theta > M_PI) theta -= 2 * M_PI;
    if (theta < -M_PI) theta += 2 * M_PI;
/*
    // update covariance: P = (I - K*H) * P
    std::array<std::array<double, 3>, 3> newP = P;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            newP[i][j] -= K[i] * P[2][j];
        }
    }
    P = newP;
    */
    // update covariance: P = (I - K*H) * P
    //the below uses full matrix multiplication for clarity
    double KH[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            KH[i][j] = K[i] * (j == 2 ? 1 : 0); // H = [0 0 1]
        }
    }

    double I_KH[3][3] = {0};
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            I_KH[i][j] = (i == j ? 1.0 : 0.0) - KH[i][j];
        }
    }

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

// full update function
void EKF::update(double leftDist, double rightDist, double imuHeading) {
    predict(leftDist, rightDist);
    updateIMU(imuHeading);
}

// getters
double EKF::getX() const { return x; }
double EKF::getY() const { return y; }
double EKF::getTheta() const { return theta; }

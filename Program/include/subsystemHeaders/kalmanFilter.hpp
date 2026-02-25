#pragma once

class KalmanFilter {
public:
    //process noise
    // increase if the filter reacts too slowly to changes, decrease if it reacts too erratically
    double q_x = 0.01;
    double q_y= 0.01;
    double q_theta = 0.1;

    //measurement noise
    //decrease if the inertial sesnor is reliable, increase if it is noisy
    double r_inertial_sensor = 0.5;

    void initialize(double x_0, double y_0, double theta_0);

    //call when resetting position
    void setPose(double x_0, double y_0, double theta_0);

    
    void update(double vertical_encoder_delta, double horizontal_encoder_delta, double heading);

    double getX(){
        return x;
    }
    double getY(){
        return y;
    }
    double getTheta(){
        return theta;
    }

private:
    double x;
    double y;
    double theta;
    double p_x;
    double p_y;
    double p_theta;
};
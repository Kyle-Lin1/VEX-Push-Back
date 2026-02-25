#include "subsystemHeaders/kalmanFilter.hpp"
#include <math.h>

//helper function
static double wrapAngle(double deg){
    deg = fmod(deg + 180.0, 360.0);
    if(deg < 0) {
        deg += 360.0;
    }
    return deg - 180.0;
}

void KalmanFilter::initialize(double x_0, double y_0, double theta_0){
    x = x_0; 
    y = y_0; 
    theta = theta_0;
    //initial covariance    
    p_x = 0.1; 
    p_y = 0.1; 
    p_theta = 1.0;
}

//call when lemlib's setPose() is used to reset position
void KalmanFilter::setPose(double x_0, double y_0, double theta_0){
    x = x_0; 
    y = y_0; 
    theta = theta_0;
    //reset covariance for confidence in the known position
    p_x = 0.1; 
    p_y = 0.1; 
    p_theta = 1.0;
}



void KalmanFilter::update(double vertical_encoder_delta, double horizontal_encoder_delta, double heading){
    double theta_rad = theta * (M_PI / 180.0);
    //prediction step
    double x_pred = x + vertical_encoder_delta * cos(theta_rad) - horizontal_encoder_delta * sin(theta_rad);
    double y_pred = y + vertical_encoder_delta * sin(theta_rad) + horizontal_encoder_delta * cos(theta_rad);
    double theta_pred = theta;
    //confidence gets worse overtime
    double p_x_pred = p_x + q_x;
    double p_y_pred = p_y + q_y;
    double p_theta_pred = p_theta + q_theta;
    //update step
    x = x_pred;
    y = y_pred;
    double k_theta = p_theta_pred / (p_theta_pred + r_inertial_sensor); 
    theta = theta_pred + k_theta * wrapAngle(heading - theta_pred); 
    //update confidence
    p_x= p_x_pred;
    p_y = p_y_pred;
    p_theta = (1.0 - k_theta) * p_theta_pred;
}
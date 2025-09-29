#pragma once
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"


// Drivetrain left motor group
extern pros::MotorGroup left_motor_group;
// Drivetrain right motor group
extern pros::MotorGroup right_motor_group;

//intake motors
extern pros::Motor lower_intake_motor;
extern pros::Motor upper_intake_motor;

//drivetrain settings
extern lemlib::Drivetrain drivetrain;

//odometry sensors
extern pros::Imu imu;
extern pros::Rotation horizontal_encoder;
extern pros::Rotation vertical_encoder;

//tracking wheels
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::TrackingWheel vertical_tracking_wheel;

//odometry settings
extern lemlib::OdomSensors odom_sensors;

//controller
extern pros::Controller controller;
extern pros::Controller partner_controller;

//input curve for throttle (forward/backward) input during driver control
extern lemlib::ExpoDriveCurve throttle_curve; 
//inout curve for steer (turning) input during driver control
extern lemlib::ExpoDriveCurve steer_curve;

// lateral PID controller
extern lemlib::ControllerSettings lateral_controller;

// angular PID controller
extern lemlib::ControllerSettings angular_controller;

//create chassis
extern lemlib::Chassis chassis;

//create potentiometer for autonomous selector
extern pros::ADIAnalogIn autonomous_selector;

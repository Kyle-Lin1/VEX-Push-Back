#include "main.h"
#include "subsystemHeaders/globals.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.hpp"

 
// Drivetrain left motor group
pros::MotorGroup left_motor_group({ -2, -11, -7 }, //motor order: front left, back left, middle left
    pros::MotorGearset::blue);
// Drivetrain right motor group
pros::MotorGroup right_motor_group({9, 19,4 }, //motor order: front right, back right, middle right
    pros::MotorGearset::blue);

//intake motors
pros::Motor lower_intake_motor(10, pros::v5::MotorGears::blue, pros::v5::MotorUnits::counts);
pros::Motor upper_intake_motor(8, pros::v5::MotorGears::rpm_200, pros::v5::MotorUnits::counts);

//scarpper pnuematic
pros::ADIDigitalOut scrapper('H');
 
//drive train settings
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10.5, // 10.5 inch track width
                              lemlib::Omniwheel::NEW_325, // anti-static 3.25" omni wheels
                              360, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
                              );


//odometry sensors               
pros::Imu imu(11);
pros::Rotation horizontal_encoder(-2);
pros::Rotation vertical_encoder(-5);

//tracking wheels
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -3.5);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_2, .22);

// odometry settings
lemlib::OdomSensors odom_sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

//controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Controller partner_controller(pros::E_CONTROLLER_PARTNER);

//input curve for throttle (forward/backward) input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, //set joystick dead zone to avoid drift
                                            10, //minimum output 
                                                1.019 //exponential curve gain
); 
//input curve for steer (turning) input during driver control
lemlib::ExpoDriveCurve steer_curve(3, //set joystick dead zone to avoid drift
                                         10, //minimum output 
                                         1.05 //exponential curve gain
); 

// lateral PID controller
lemlib::ControllerSettings lateral_controller(6, // proportional gain (kP)
                                              0.03, // integral gain (kI)
                                              .8, // derivative gain (kD)
                                              6, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              11.9, // derivative gain (kD)
                                                3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

//create chassis
lemlib::Chassis chassis(
                        drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        odom_sensors // odometry sensors
);

//create potentiometer for autonomous selector
pros::ADIAnalogIn autonomous_selector('A');

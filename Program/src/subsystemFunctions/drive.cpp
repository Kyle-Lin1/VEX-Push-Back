#include "subsystemHeaders/drive.hpp"
#include "main.h"
#include "subsystemHeaders/globals.hpp"
#include <math.h>

//helper functions
double avgMotorEncoders(){
        double totalPosition = left_motor_group.get_position() + right_motor_group.get_position();
    return totalPosition / 2;
}
double avgEncoderValues(){
        double totalPosition = left_motor_group.get_position() + right_motor_group.get_position();
    return totalPosition / 2;
}
void resetEncoderValues(){
        left_motor_group.tare_position();
        right_motor_group.tare_position();
}

// opcontrol functions
void set_drive(){
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        //limit maximum turning power for precision
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)*.85;

        //create dead zone to prevent controller drift
        if(abs(leftY) < 5){
                leftY = 0;
        }
        if(abs(rightX) < 10){
                rightX = 0;
        }

        // move the robot
       chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(20);
}

//autonomous functions
void turn(double millisec, int power){
        left_motor_group.move(power);
        right_motor_group.move(-power);
        pros::delay(millisec);
        left_motor_group.move(0);
        right_motor_group.move(0);
}
void move(int power, double milliseconds){
        left_motor_group.move(power);
        right_motor_group.move(power);
        pros::delay(milliseconds);
        right_motor_group.move(0);
        left_motor_group.move(0);
}

void translate(double distance, int timeout, double maxSpeed) {
    double kP = 9; // Proportional gain, adjust as necessary
    double kI = 0; // Integral gain, adjust as necessary
    double kD = 9.3; // Derivative gain, adjust as necessary

    double error = distance;
    double integral = 0;
    double previous_error = 0;
    int startTime = pros::millis();

    vertical_encoder.reset();

    while (pros::millis() - startTime < timeout) {
        double current_position = vertical_encoder.get_position() / 36000.0 * (2.125 * M_PI); // Convert encoder units to distance
        error = distance - current_position;
        integral += error;
        double derivative = error - previous_error;

        double power = kP * error + kI * integral + kD * derivative;

        // Limit the power to the maximum speed
        if (power > maxSpeed) {
            power = maxSpeed;
        } else if (power < -maxSpeed) {
            power = -maxSpeed;
        }

        left_motor_group.move(power);
        right_motor_group.move(power);

        previous_error = error;

        // Check if the robot is close enough to the target
        if (fabs(error) < 1) {
            break;
        }

        pros::delay(20); // Delay to prevent hogging CPU
    }

    // Stop the motors
    left_motor_group.move(0);
    right_motor_group.move(0);
}


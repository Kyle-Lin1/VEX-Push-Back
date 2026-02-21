#include "subsystemHeaders/drive.hpp"
#include "main.h"
#include "subsystemHeaders/globals.hpp"
#include <math.h>

//helper functions
 
void setDrive(int left, int right){
    left_motor_group.move(left);
    right_motor_group.move(right);
}
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
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)*.65;

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

void wiggle(int millisec){
        int frequency = 100;
        int wiggles = millisec/frequency;
        
        for(int i = 0; i < wiggles; i++){
                left_motor_group.move(50);
                right_motor_group.move(-50);
                pros::delay(frequency/2);
                left_motor_group.move(-50);
                right_motor_group.move(50);
                pros::delay(frequency/2);
        }
        left_motor_group.move(0);
        right_motor_group.move(0);
}

void turn(double millisec, int power){
        left_motor_group.move(power);
        right_motor_group.move(-power);
        pros::delay(millisec);
        left_motor_group.move(0);
        right_motor_group.move(0);
}
void move(int voltage, double milliseconds){
        left_motor_group.move(voltage);
        right_motor_group.move(voltage);
        pros::delay(milliseconds);
        right_motor_group.move(0);
        left_motor_group.move(0);
}



void translate(int inch, int voltage){
    //define direction based on units provided
    int direction = abs(inch)/inch; //either 1 or -1
    //reset motor encoders 
    resetEncoderValues();
    imu.tare_heading(); //reset gyroscope heading
    //drive forward until units are reached
    while(avgEncoderValues() < ((fabs(inch) * 1800) / (M_PI * 3.25 *.6))){ //convert inches to motor units
        //Note: scale inertial sensor heading for over or under correcting
        setDrive(voltage * direction + imu.get_heading()*10, voltage * direction - imu.get_heading()*10); 
        pros::delay(10);
    }
    //brief manual brake
    setDrive(-10 * direction,-10 * direction);
    pros::delay(50);//change millisec depending on weight of robot
    //set drive back to nuetral
    setDrive(0,0);
}

//function for turning
void rotate(int degrees, int voltage){
    //define direction based on the units provided
    //if turn left use positive number, if right use negative number
    int direction = abs(degrees)/degrees;
    //resetting the gyroscope
    imu.tare_heading();
    //turn intil units are reached
    setDrive(-voltage * direction, voltage * direction);
    //turn until units - 5 degrees is reached
    //Note: subract a little from target degrees to anticipate over turning (hopefully saves time)
    while(fabs(imu.get_heading()) < abs(degrees) - 5){ 
        pros::delay(10);
    }
    //let robot lose momentum
    setDrive(0,0);
    pros::delay(100); //wait until the robot completely stops (based on weight)
    //correct over or under shoot
    if(fabs(imu.get_heading()) > abs(degrees)){
        setDrive(.5 * voltage * direction, .5 * -voltage * direction);
        while(fabs(imu.get_heading()) > abs(degrees)){
            pros::delay(10);
        } 
    }else if (fabs(imu.get_heading()) < abs(degrees)){
        setDrive(.5 * -voltage * direction, .5 * voltage * direction);
        //Note: subract a little from target degrees to anticipate over turning (hopefully saves time)
        while(fabs(imu.get_heading()) < abs(degrees) - 5){ 
            pros::delay(10);
        }
    }
    //reset drive to zero
    setDrive(0,0);
}
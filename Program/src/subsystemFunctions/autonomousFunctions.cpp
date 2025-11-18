#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "liblvgl/llemu.hpp"
#include "liblvgl/misc/lv_async.h"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "subsystemHeaders/drive.hpp"
#include "subsystemHeaders/globals.hpp"
#include "subsystemHeaders/autonomousHeaders.hpp"
#include <chrono>
#include <memory>
#include "subsystemHeaders/intake.hpp"
#include <random>

void tuning(){
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0, 24, 10000000,{.maxSpeed = 60});
}

void skillsParking(){
    lower_intake_motor.move(-127);
    move(60, 1000);
    move(-40, 500);
}

void redRight(){
    lower_intake_motor.move(127);
    scrapper.set_value(true);
    move(60, 950);
    pros::delay(500);
    turn(360, 60);
    pros::delay(100);
    move(40, 1600);
    left_motor_group.move(30);
    right_motor_group.move(30);
    //slam wall 6 times to ensure blocks are removed
    for (int i=0;i<4;i++){
        move(40,300);
        move(-40,100);
    }
    left_motor_group.move(-15);
    right_motor_group.move(-15);
    pros::delay(500);
    move(-40, 1400);
    upper_intake_motor.move(127);
}

void redLeft() {
    lower_intake_motor.move(127);
    scrapper.set_value(true);
    move(60, 975);
    pros::delay(500);
    turn(360, -60);
    pros::delay(100);
    move(40, 1600);
    left_motor_group.move(30);
    right_motor_group.move(30);
    //slam wall 6 times to ensure blocks are removed
    for (int i=0;i<4;i++){
        move(40,300);
        move(-40,100);
    }
    turn(30, -60);
    left_motor_group.move(-15);
    right_motor_group.move(-15);
    pros::delay(500);
    move(-40, 1400);
    upper_intake_motor.move(127);
}


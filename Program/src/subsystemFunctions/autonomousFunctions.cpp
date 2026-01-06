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


void skills(){
    chassis.setPose(23,-17.5,180);
    lower_intake_motor.move(127);
    scrapper.set_value(true);
    //slam into match loader
    chassis.moveToPoint(23, -48, 1500, {.maxSpeed = 80, .minSpeed = 30});
    chassis.turnToHeading(270, 500);
    chassis.moveToPoint(0, -48, 1500, {.maxSpeed = 80, .minSpeed = 30});
    //score in long goal
    chassis.moveToPoint(15, -48, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30}, false);
    //remove blue blocks
    chassis.turnToHeading(120, 500);
    chassis.moveToPose(48, -60, 90, 2000, {.forwards =false, .maxSpeed = 80, .minSpeed = 30});
}

void redRight_guardGoal(){
    chassis.setPose(24,-12,120.2);
    lower_intake_motor.move(127);
    //move to the first three blocks
    chassis.moveToPoint(49, -22, 1000, {.maxSpeed = 60, .earlyExitRange = 8});

    //move to nuetral blocks
    chassis.moveToPose(64, -47, 154, 2500, {.maxSpeed = 60, .earlyExitRange = 8}, false);
    chassis.turnToHeading(166, 60);
    scrapper.set_value(true);
    chassis.moveToPoint(55, -30, 1600, {.forwards=false, .maxSpeed = 70, .earlyExitRange = 8}, false);
    chassis.turnToHeading(270, 700); 
    chassis.moveToPoint(30, -46, 2000, {.maxSpeed = 80, .minSpeed = 30,  .earlyExitRange = 8}, false);
    //slam into match loader
    chassis.moveToPose(6, -47, 270, 2500, {.maxSpeed = 80, .minSpeed = 30});
    pros::delay(2500);
    //score in long goal
    chassis.moveToPoint(60, -48, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30}, false);
    pros::delay(500);
    upper_intake_motor.move(127);
    pros::delay(2500);
    //guard goal
    chassis.setPose(60, -48, 270);    
    chassis.moveToPose(55, -48, 270, 500, { .maxSpeed = 80, .minSpeed = 30});
    chassis.moveToPoint(70, -48, 500, {.forwards = false});
}

void redRight_clearMatchLoad(){
    chassis.setPose(24,-12,120.2);
    lower_intake_motor.move(127);
    //move to the first three blocks
    chassis.moveToPoint(48, -22, 1000, {.maxSpeed = 60, .earlyExitRange = 8});
    //move to nuetral blocks
    chassis.moveToPose(64, -47, 154, 2500, {.maxSpeed = 60, .earlyExitRange = 8}, false);
    chassis.turnToHeading(166, 60);
    scrapper.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(55, -30, 1600, {.forwards=false, .maxSpeed = 70, .earlyExitRange = 8}, false);
    chassis.turnToHeading(270, 700); 
    chassis.moveToPoint(30, -46, 2000, {.maxSpeed = 80, .minSpeed = 30,  .earlyExitRange = 8}, false);
    //slam into match loader
    chassis.moveToPose(6, -48, 270, 2200, {.maxSpeed = 80, .minSpeed = 30});
    //score in long goal
    chassis.moveToPoint(60, -48, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30}, false);
    pros::delay(500);
    upper_intake_motor.move(127);
    pros::delay(2400);
    //remove blue blocks
    chassis.setPose(40, -48, 270);    
    chassis.moveToPose(0, -46, 275, 2000, {.maxSpeed = 80, .minSpeed = 30});
}

/*
void redRight(){
    chassis.setPose(24,-12,120.2);
    lower_intake_motor.move(127);
    //move to the first three blocks
    chassis.moveToPoint(48, -22, 1000, {.maxSpeed = 60, .earlyExitRange = 8});
    //move to nuetral blocks
    chassis.moveToPose(67, -52, 164, 1500, {.maxSpeed = 100}, false);
    pros::delay(100);
    scrapper.set_value(true);
    pros::delay(100);
    chassis.moveToPoint(55, -35, 1500, {.forwards=false, .maxSpeed = 70, .earlyExitRange = 8}, false);
    chassis.turnToHeading(270, 500); 
    chassis.moveToPoint(30, -48, 2000, {.maxSpeed = 80, .minSpeed = 30,  .earlyExitRange = 8}, false);
    //slam into match loader
    chassis.moveToPose(0, -50, 270, 1500, {.maxSpeed = 80, .minSpeed = 30});
    //score in long goal
    chassis.moveToPoint(60, -48, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30}, false);
    pros::delay(500);
    upper_intake_motor.move(127);
    pros::delay(2500);
    //remove blue blocks
    chassis.setPose(40, -48, 270);    
    chassis.moveToPose(0, -48, 270, 2000, {.maxSpeed = 80, .minSpeed = 30});
}
    */

void skillsParking(){
    lower_intake_motor.move(-127);
    move(60, 1000);
    move(-40, 500);
}

void oldRedRight(){
    lower_intake_motor.move(127);
    scrapper.set_value(true);
    move(60, 950);
    pros::delay(500);
    turn(360, 60);
    pros::delay(100);
    move(40, 1000);
    //left_motor_group.move(30);
    //right_motor_group.move(30);
    //slam wall
    pros::delay(00);
    left_motor_group.move(-15);
    right_motor_group.move(-15);
    pros::delay(500);
    move(-40, 1400);
    move(40,100);
    upper_intake_motor.move(127);
}

void oldRedLeft() {
    lower_intake_motor.move(127);
    scrapper.set_value(true);
    move(60, 975);
    pros::delay(500);
    turn(360, -60);
    pros::delay(100);
    //slam wall
    move(40, 1000);
    turn(30, -60);
    left_motor_group.move(-15);
    right_motor_group.move(-15);
    scrapper.set_value(false);
    pros::delay(500);
    move(-40, 1400);
    move(40,100);
    upper_intake_motor.move(127);
}


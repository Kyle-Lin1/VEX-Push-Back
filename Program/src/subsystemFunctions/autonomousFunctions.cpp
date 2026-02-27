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
#include "subsystemHeaders/poseControl.hpp"
#include <chrono>
#include <memory>
#include "subsystemHeaders/intake.hpp"
#include <random>

void skillsParking(){
    setRobotPose(20.5,-17.5,180);
    
    lower_intake_motor.move(127);
    scrapper.set_value(true);
    chassis.moveToPoint(23, -48, 1500, {.maxSpeed = 80, .minSpeed = 30});
    chassis.turnToHeading(270, 500);
    //slam into match loader
    chassis.moveToPoint(0, -48, 1000, {.maxSpeed = 40, .minSpeed = 30});
    chassis.moveToPoint(16, -48, 1000, {.forwards = false, .minSpeed = 30}, false);
    chassis.moveToPoint(0, -48, 2000, {.maxSpeed = 40, .minSpeed = 30}, false);
    chassis.turnToHeading(265, 500);
    chassis.moveToPoint(60, -48, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30}, false);
    pros::delay(500);
    upper_intake_motor.move(127);
    pros::delay(5000);
    chassis.moveToPoint(30, -48, 500, {.maxSpeed = 80, .minSpeed = 30}, false);
    chassis.turnToHeading(0, 500 );
    lower_intake_motor.move(-127);
    chassis.moveToPose(8, 2, 0, 1000, {.maxSpeed = 80, .minSpeed = 30}, false);
}

void skills(){
    setRobotPose(20.5,-17.5,180);
    lower_intake_motor.move(127);
    scrapper.set_value(true);
    //slam into match loader
    chassis.moveToPoint(20, -46, 1500, {.maxSpeed = 70, .minSpeed = 30});
    chassis.turnToHeading(270, 500);
    chassis.moveToPoint(0, -49.5, 3500, {.maxSpeed = 40, .minSpeed = 30});
    //chassis.moveToPoint(10, -49, 500, {.forwards = false, .minSpeed = 25}, false);
    //chassis.moveToPoint(0, -51, 1500, {.maxSpeed = 40, .minSpeed = 30}, false);
    //chassis.moveToPoint(37.5, -38, 1000, {.forwards = false, .minSpeed = 70, .earlyExitRange = 6});
    chassis.moveToPoint(24, -48, 1000, {.forwards = false}, false);
    chassis.turnToHeading(135, 500);
    lower_intake_motor.move(0);
    scrapper.set_value(false);
    chassis.moveToPoint(37.5, -62, 1500, {.minSpeed = 70, .earlyExitRange = 6}, false);
    chassis.turnToHeading(90, 200);
    chassis.moveToPoint(120, -63.5, 2500, {.maxSpeed = 75});
    chassis.turnToHeading(180, 700);
    chassis.moveToPoint(120, -50, 700, {.forwards = false});
    //chassis.moveToPoint(120, -38, 2000, {.forwards=false});
    chassis.turnToHeading(90, 600);
    //chassis.moveToPoint(120, -52, 800, {.forwards=false});
    chassis.moveToPose(100, -50, 90, 2000, {.forwards=false,}, false);
    lower_intake_motor.move(-127);
    upper_intake_motor.move(127);
    pros::delay(300);
    lower_intake_motor.move(127);
    pros::delay(3000);
    setRobotPose(103,-48, chassis.getPose().theta);
    upper_intake_motor.move(0);
    scrapper.set_value(true);
    //chassis.moveToPoint(150, -48, 1500, {.maxSpeed=50});
    chassis.moveToPoint(150, -48, 3700, {.maxSpeed=50});
    chassis.moveToPoint(94, -48, 1500, {.forwards=false, .maxSpeed=80}, false);
    lower_intake_motor.move(-127);
    upper_intake_motor.move(127);
    pros::delay(400);
    lower_intake_motor.move(127);
    pros::delay(3000);
    setRobotPose(103,-48, chassis.getPose().theta);
    upper_intake_motor.move(0);
    chassis.moveToPoint(110, -48, 1000);
    //chassis.moveToPoint(110, -30, 1000, {.minSpeed=40, .earlyExitRange=5});
    //chassis.moveToPoint(110, 24, 3000, {.minSpeed=40, .earlyExitRange=5});
    chassis.moveToPoint(115, 52.5, 3000);
    chassis.turnToHeading(90, 1500);
    chassis.moveToPoint(150, 53.5, 3500, {.maxSpeed=50});
    chassis.moveToPoint(120, 53.5, 1000, {.forwards=false}, false);
    chassis.turnToHeading(315, 500);
    scrapper.set_value(false);
    chassis.moveToPoint(107.5, 66, 1100, {.minSpeed = 70, .earlyExitRange = 6});
    lower_intake_motor.move(0);
    chassis.moveToPoint(24, 66, 3000, {.maxSpeed = 70});
    chassis.turnToHeading(0, 600);
    chassis.moveToPoint(24, 55.5, 1000, {.forwards=false});
    chassis.moveToPose(48, 55.5, 270, 2000, {.forwards=false}, false);
    lower_intake_motor.move(-127);
    upper_intake_motor.move(127);
    pros::delay(300);
    lower_intake_motor.move(127);
    pros::delay(3000);
    setRobotPose(41,48,chassis.getPose().theta);
     upper_intake_motor.move(0);
     scrapper.set_value(true);
    //chassis.moveToPoint(0, 48, 1500, {.maxSpeed=50});
    chassis.moveToPoint(0, 47, 3500, {.maxSpeed=50});
    chassis.moveToPoint(60, 48, 1500, {.forwards=false, .maxSpeed=80}, false);
    lower_intake_motor.move(-127);
    upper_intake_motor.move(127);
    pros::delay(400);
    lower_intake_motor.move(127);
    pros::delay(3000);
    setRobotPose(41,48, chassis.getPose().theta);
    upper_intake_motor.move(0);
    lower_intake_motor.move(0);
    scrapper.set_value(false);
    chassis.moveToPose(8, 15, 180, 1500, {.maxSpeed=127}, false);
    hook.set_value(true);
    upper_intake_motor.move(127);
    lower_intake_motor.move(127);
    scrapper.set_value(true);
    chassis.moveToPoint(4, 0, 3000, {.maxSpeed=100}, false);
    scrapper.set_value(false);
}

void redLeft(){
    setRobotPose(20,14,55);
    lower_intake_motor.move(127);
    intake_piston.set_value(true);
    chassis.moveToPoint(30, 18, 1500,{.maxSpeed=80, .minSpeed=20, .earlyExitRange=4});
    chassis.moveToPoint(53, 25, 1500, {.maxSpeed=80});
    chassis.moveToPose(63, 12, 315, 2000,{.forwards=false}, false);
    upper_intake_motor.move(127);
    pros::delay(300);
    upper_intake_motor.move(0);
    pros::delay(300);
    upper_intake_motor.move(127);
    pros::delay(1000);
    intake_piston.set_value(false);
    upper_intake_motor.move(0);
    chassis.moveToPoint(30, 49, 1500, {.maxSpeed = 80, .minSpeed = 30});
    scrapper.set_value(true);
    chassis.turnToHeading(270, 1000); 
    chassis.moveToPoint(-1, 52, 1800, {.maxSpeed = 40, .minSpeed = 30});
    chassis.moveToPoint(50, 53, 1500, {.forwards=false, .maxSpeed=80}, false);
    lower_intake_motor.move(-127);
    upper_intake_motor.move(127);
    pros::delay(300);
    lower_intake_motor.move(127);
    pros::delay(3000);
}

void redRight_guardGoal(){
    setRobotPose(24,-12,120.2);
    /*
    lower_intake_motor.move(127);
    hook.set_value(true);
    //move to the first three blocks
    chassis.moveToPoint(49, -22, 1000, {.maxSpeed = 40, .earlyExitRange = 8});
    //move to nuetral blocks
    chassis.moveToPose(64, -47, 154, 2500, {.maxSpeed = 60, .earlyExitRange = 8}, false);
    chassis.turnToHeading(166, 60, {}, true);
    scrapper.set_value(true);
    chassis.moveToPoint(55, -30, 1600, {.forwards=false, .maxSpeed = 70, .earlyExitRange = 8}, false);
    chassis.turnToHeading(270, 700); 
    chassis.moveToPoint(30, -46, 2000, {.maxSpeed = 80, .minSpeed = 30,  .earlyExitRange = 8}, false);
    //slam into match loader
    chassis.moveToPose(6, -47, 270, 2550, {.maxSpeed = 80, .minSpeed = 30});
    pros::delay(2500);
    //score in long goal
    chassis.moveToPoint(60, -48, 1000, {.forwards = false, .maxSpeed = 80, .minSpeed = 30}, false);
    pros::delay(500);
    upper_intake_motor.move(127);
    pros::delay(2500);
    //guard goal
    setRobotPose(60, -48, 270);
    chassis.moveToPose(55, -48, 270, 500, { .maxSpeed = 80, .minSpeed = 30});
    chassis.moveToPoint(70, -48, 500, {.forwards = false});
    */
}

void redRight_clearMatchLoad(){
    setRobotPose(20,-14,125);
    lower_intake_motor.move(127);
    chassis.moveToPoint(30, -18, 1500,{.maxSpeed=80, .minSpeed=20, .earlyExitRange=4}, false);
    chassis.moveToPoint(53, -25, 1500, {.maxSpeed=80},false);
    chassis.moveToPoint(61.5, -44, 1500,{.maxSpeed=75});

    chassis.moveToPoint(48, -24, 1500, {.forwards=false});
    chassis.moveToPoint(23, -48, 1500, {.maxSpeed = 80, .minSpeed = 30});
    scrapper.set_value(true);
    chassis.moveToPoint(-15, -48, 2000, {.maxSpeed = 40, .minSpeed = 30});
    chassis.moveToPoint(55, -48, 1500, {.forwards=false, .maxSpeed=80}, false);
    lower_intake_motor.move(-127);
    upper_intake_motor.move(127);
    scrapper.set_value(false);
    pros::delay(300);
    lower_intake_motor.move(127);
    pros::delay(3000);
}

/*
void redRight(){
    setRobotPose(24,-12,120.2);
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
    setRobotPose(40, -48, 270);
    chassis.moveToPose(0, -48, 270, 2000, {.maxSpeed = 80, .minSpeed = 30});
}
    */
/*
void skillsParking(){
    lower_intake_motor.move(-127);
    move(60, 1000);
    move(-40, 500);
}
*/
void oldRedRight(){
    lower_intake_motor.move(127);
    scrapper.set_value(true);
    move(60, 1050);
    pros::delay(500);
    turn(360, 60);
    pros::delay(100);
    move(40, 2000);
    //left_motor_group.move(30);
    //right_motor_group.move(30);
    //slam wall
    move(-40, 300);
    move(40, 1500);
    left_motor_group.move(-15);
    right_motor_group.move(-15);
    pros::delay(1000);
    move(-40, 1400);
    //move(40,100);
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


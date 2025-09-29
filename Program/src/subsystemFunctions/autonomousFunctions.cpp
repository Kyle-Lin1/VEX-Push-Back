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
#include <random>


void testAuton() {
    move(60, 1000);
    pros::delay(1000);
    /*
    move(60, 500);
    pros::delay(50);
    move(-60, 500);
    pros::delay(50);
    turn(500, 60);
    pros::delay(100);
    move(60, 1000);
    pros::delay(500);
    turn(500, 60);
    */
}

void redRight() {
    chassis.setPose(1, 1, 90);
}
#pragma once

//helper functions
double avgMotorEncoders();
void move(int power, double milliseconds);
double avgEncoderValues();
void resetEncoderValues();

// opcontrol functions
void set_drive();

//autonomous functions
void drive(double enoderUnits, int power);
void turn(double millisec, int power);
void translate(double distance, int timeout, double maxSpeed);
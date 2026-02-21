#pragma once

//helper functions
void setDrive(int left, int right);
double avgMotorEncoders();
void move(int voltage, double milliseconds);
double avgEncoderValues();
void resetEncoderValues();

// opcontrol functions
void set_drive();

//autonomous functions
void wiggle(int millisec);
void drive(double enoderUnits, int power);
void turn(double millisec, int power);
void translate(double distance, int timeout, double maxSpeed);
void rotate(int degrees, int voltage); 
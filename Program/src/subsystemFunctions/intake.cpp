#include "main.h"
#include "subsystemHeaders/intake.hpp"

void set_lower_intake(int power){
        //This is for blue motors 
        lower_intake_motor.move_velocity(power*4.72440944882); //(percent velocity as decimal)*(range of motor)/(voltage) = (1)(600)/127

        lower_intake_motor.move(power);
}

void set_upper_intake(int power){
        //This is for 5.5 watt motors
        //5.5 watt motors have 200 rpm, which is the same as green motors
        upper_intake_motor.move_velocity(power*1.57480314961); //(percent velocity as decimal)*(range of motor)/(voltage) = (1)(200)/127

        upper_intake_motor.move(power);
} 

void set_intake() {
       int lowerMotorPower = 127 * (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)
    - controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

        int upperMotorPower = 127 * (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)
    - controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2));

    set_upper_intake(upperMotorPower);
    set_lower_intake(lowerMotorPower);
}
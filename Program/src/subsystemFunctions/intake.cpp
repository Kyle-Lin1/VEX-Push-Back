#include "main.h"
#include "subsystemHeaders/intake.hpp"

void set_intake_motors(int power){
        lower_intake_motor.move_velocity(power*4.72440944882); //(percent velocity)*(range of motor)/(voltage)
        lower_intake_motor.move(power);
}

void set_intake() {
       int motorPower = 127 * (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)
    - controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

    set_intake_motors(motorPower);
}
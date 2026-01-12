#include "main.h"
#include "subsystemHeaders/hook.hpp"

bool set_hook(bool hook_state){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        if (hook_state == true){//if solenoid is not activated, activate it
            hook.set_value(true);
            hook_state = false;
        }
        else if (hook_state == false){
            hook.set_value(false); //if solenoid is activated, deactivate it
            hook_state = true;
        }
    }
    pros::delay(20); //delay to save resources
    return hook_state;
}
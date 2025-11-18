#include "main.h"
#include "subsystemHeaders/scrapper.hpp"

bool set_scrapper(bool scrapper_state){
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
        if (scrapper_state == true){//if solenoid is not activated, activate it
            scrapper.set_value(true);
            scrapper_state = false;
        }
        else if (scrapper_state == false){
            scrapper.set_value(false); //if solenoid is activated, deactivate it
            scrapper_state = true;
        }
    }
    pros::delay(20); //delay to save resources
    return scrapper_state;
}           
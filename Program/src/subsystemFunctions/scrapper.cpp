#include "main.h"
#include "subsystemHeaders/scrapper.hpp"

bool set_scrapper(bool scrapper_state){
    if (pros::E_CONTROLLER_ANALOG_LEFT_Y){
        if (!scrapper_state){//if solenoid is not activated, activate it
            scrapper.set_value(true);
        }
        else{
            scrapper.set_value(false); //if solenoid is activated, deactivate it
        }
    }
}
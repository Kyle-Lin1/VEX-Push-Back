#include "main.h"
#include "subsystemHeaders/autonomousSelector.hpp"

int get_auton_selector(int auton_count) {
    // Read the potentiometer value
    int pot_value = autonomous_selector.get_value();
    
    // Map the potentiometer value to an autonomous selector index
    // Potentiometer value ranges from 0 to 4095
    int index = (pot_value * auton_count) / 4096;

    return index;
}
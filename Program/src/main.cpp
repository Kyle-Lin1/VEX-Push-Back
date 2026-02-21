#include "main.h"
//prevent unhelpful unused include warnings
#include "lemlib/api.hpp" // IWYU pragma: keep 
#include "subsystemHeaders/drive.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "32092D");
	chassis.calibrate(); // calibrate sensors
	pros::delay(2000); // wait for calibration to finish
	pros::lcd::register_btn1_cb(on_center_button);
	//thread to for brain screen and position logging
	pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
// Define auton_names array with names of autonomous routines


void autonomous() {
	//hold for accuracy
	left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	//tuning();
	redLeft();
	//skillsParking();
	//redRight_clearMatchLoad();
	//skillsParking();
	//skills();
	//oldRedRight();
	/*
	// Add names of autonomous routines here
	// Make sure the order matches the autons array below
	const char* auton_names[] = {
	"redLeft",
	"redRight",
	"skills"
	};

	// Array of function pointers
	void (*autons[])() = {// Add autonomous functions here
		//redleft,
		//redRight,
		//skills
	}; 

	// calls auton selector function
	// auton_count is automatically inputted by dividing memory size of the array by the size of a single function pointer
	int auton_index = get_auton_selector(sizeof(autons) / sizeof(autons[0])); 

	// Print name of the selected auton
	pros::lcd::print(0, "Selected: %s", auton_names[auton_index]);

	// ---- Run the chosen auton ----
	autons[auton_index]();
	*/
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	//set brake mode to brake
  	left_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  	right_motor_group.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	//false = not activated, true = activated
	bool scrapper_state = false; //initialize scrapper state 
	bool intake_state = false; //initialize intake piston state
	bool hook_state = false; //initialize hook state
	while (true) {
		set_drive(); // set drive controls
		set_intake(); //set intake controls
		scrapper_state = set_scrapper(scrapper_state);
		intake_state = set_intake_piston(intake_state);
		hook_state = set_hook(hook_state);
		pros::delay(20);  // 20 second delay to save resources
	}
}
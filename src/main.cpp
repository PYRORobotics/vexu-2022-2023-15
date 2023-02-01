#include "main.h"
#include "navX.h"
#include "AMT21.h"


/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
void autonomous() {}

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
	pros::Controller master(pros::E_CONTROLLER_MASTER);
    navX navx(13);
    bool foo = true;
    AMT21 amt21_left(19, 0x58);
    AMT21 amt21_right(19, 0x5C);
    AMT21 amt21_middle(19, 0x54);
    pros::delay(20);
    uint64_t startTime = 0;
    while(true){
        pros::delay(10);
        startTime = pros::micros();
        /*int position_left = amt21_left.get_position();
        //while(pros::micros() -startTime < 1000)
        //pros::delay(1);
        int turns_left = amt21_left.get_turns();
        //while(pros::micros() -startTime < 2000);
        //pros::delay(1);
        int position_right = amt21_right.get_position();
        //while(pros::micros() -startTime < 3000);
        //pros::delay(1);
        int turns_right = amt21_right.get_turns();
        //pros::delay(1);
        //while(pros::micros() -startTime < 4000);
        int position_middle = amt21_middle.get_position();
        //while(pros::micros() -startTime < 5000);
        int turns_middle = amt21_middle.get_turns();
        //while(pros::micros() -startTime < 6000);*/

        /*pros::lcd::print(0, "AMT21_left position: %d", position_left);
        pros::lcd::print(1, "AMT21_left turns: %d", turns_left);
        pros::lcd::print(2, "AMT21_right position: %d", position_right);
        pros::lcd::print(3, "AMT21_right turns: %d", turns_right);
        pros::lcd::print(4, "AMT21_middle position: %d", position_middle);
        pros::lcd::print(5, "AMT21_middle turns: %d", turns_middle);*/

        pros::lcd::print(0, "AMT21_left value: %d", amt21_left.get_value());
        pros::lcd::print(2, "AMT21_right value: %d", amt21_right.get_value());
        pros::lcd::print(4, "AMT21_middle value: %d", amt21_middle.get_value());

        pros::lcd::print(6, "Response time: %ld micros", pros::micros() - startTime);
        pros::lcd::print(7, "heading: %f", navx.getHeading());
    }
}

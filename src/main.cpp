#include "main.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "okapi/impl/device/rotarysensor/IMU.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "robotControl.h"
#include "cartesian.h"
#include "polar.h"
#include "odom.h"
#include "path.h"

using okapi::inch;

pros::Controller master(pros::E_CONTROLLER_MASTER);

/*----------------------------------------------------------------------\
|  Name: Emily Sanders													|
|  Major: Software Engineering											|
|  Fun Fact: I've been doing robotics since around 5th grade :D			|
\----------------------------------------------------------------------*/

//Andrew Kang-Mechanical Engineering-I swam in high school
/*
 * Charles Jeffries
 * Software Engineering Major, Senior
 * I've been doing VEX for 9 years, and I also have 9 monitors connected to my desktop at home!
 */


/**
 *
 *
 * Dominic Murdica
 * Software engineering
 * I am Scuba certified, and enjoy dirt bike riding, rock climbing, and hunting.
 */

/*
My name is Andrew Ayerh, 
I'm majoring in Computer Science, 
and I like to play volleyball in my free time.
*/

/*
Taz Hellman, Software Engineering Major, my brother is 16 years older than me
*/


//Name: Brian Levenseller
//Major Software Engineering
//Fun Fact: I am remarkably bad at thinking of fun facts

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

 //William Reinhart, Electrical Engineering, I played hockey for 15 years

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
	//

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
void autonomous() {
	RobotControl robot1;
	imu1.reset();
	while(imu1.is_calibrating()==true) {
		pros::delay(20);
	}
	pros::delay(100);
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

double normLeftX() { 
	return master.get_analog(ANALOG_LEFT_X)/1.27;
}
double normLeftY() {
	return master.get_analog(ANALOG_LEFT_Y)/1.27;
}
double normRightX() {
	return master.get_analog(ANALOG_RIGHT_X)/1.27;
}
double normRightY() {
	return master.get_analog(ANALOG_RIGHT_Y)/1.27;
}



// double powerMod = 0.3;
// double BALLS = 0;
// void raw_tank(double straightPow, double turnPow, double strafePow) {
// 	left_front_mtr = (straightPow + turnPow + strafePow)*powerMod;
// 	right_front_mtr = (straightPow - turnPow - strafePow)*powerMod;
// 	left_back_mtr = (straightPow + turnPow - strafePow)*powerMod;
// 	right_back_mtr = (straightPow - turnPow + strafePow)*powerMod;
// 	if((pros::millis()-BALLS)>2000000000) {
// 		printf("left front: %f\n", (straightPow + turnPow + strafePow)*powerMod);
// 		printf("right front: %f\n", (straightPow - turnPow - strafePow)*powerMod);
// 		printf("right front: %f\n", (straightPow + turnPow - strafePow)*powerMod);
// 		printf("right front: %f\n", (straightPow - turnPow + strafePow)*powerMod);
// 		printf("-------------------------\n");
// 		BALLS=pros::millis();
// 	}

// }

float diagnosticTimer;
void opcontrol() {
	std::vector<robotPose> Test = Path::qbezierManualHeading(robotPose(Cartesian(0.0,0.0) , okapi::QAngle(90.0)) , robotPose(Cartesian(24.0,48.0) , okapi::QAngle(40.0)) , robotPose(Cartesian(48.0,0.0) , okapi::QAngle(240.0)) , 100);
	int progress = 0;
	RobotControl robot1;
	//AMT21 amt21_left(19, 0x58);
	imu1.reset();
	while(imu1.is_calibrating()==true) {
		pros::delay(20);
	}
	pros::delay(100);
	Cartesian stick1;
	okapi::QAngle robotHeading = 0_rad;
	odom odom1 = odom();
	double drivePow=0;
	pros::Motor intake ();
	while (true) {
		//pros::lcd::print(0, "AMT21_left value: %d", amt21_left.get_value());
        pros::lcd::print(2, "AMT21_right value: %d", up.get_value());
        pros::lcd::print(4, "AMT21_middle value: %d", sideways.get_value());
        //pros::lcd::print(6, "Response time: %ld micros", pros::micros() - start);
        //pros::lcd::print(7, "heading: %f", navx.getHeading());
		odom1.updateOdom();
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		(pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		(pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		int x = master.get_analog(ANALOG_LEFT_X);
		int y = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		stick1.updatePoint(x*1_in, y*1_in);
		okapi::QAngle headingInput = stick1.getHeading();
		double magn = pow((x*x+y*y), 0.5);
		if(fabs(normRightX())>10) {
			robotHeading = robotHeading+normRightX()*.06_deg;
		}
		if((fabs(normRightX())<5)&&(fabs(robotHeading.convert(okapi::degree)-imu1.get_heading())>50)) {
			robotHeading = imu1.get_heading()*1_deg;
		}
		if((pros::millis()-diagnosticTimer)>1000) {
            // printf("magn: %f\n", magn);
            // printf("turn:  %d\n", turn);
			// printf("controller: %f\n", normRightX());
			// printf("robotHeading: %f\n", robotHeading.convert(okapi::degree));
            // printf("-------------------------\n");
			printf("up: %d\n", up.get_value());
			printf("sideways: %d\n", sideways.get_value());
            diagnosticTimer = pros::millis();
			odom1.printOdom();
		}
		// robot1.absStrafe(headingInput, magn, turn*1);
		 
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)==true) {
			drivePow = odom1.position.getMagnitude().convert(okapi::inch)*30;
			if (drivePow>100) {
				drivePow = 100;
			}
			robotHeading = 0_rad;
			robot1.headingStrafe(odom1.position.getHeading()+180_deg, drivePow, 0_rad);	
		} else {
			robot1.headingStrafe(headingInput, magn, robotHeading);	
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			odom1.resetOdom();
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
			//intake.move(12);
			for(std::vector<robotPose>::iterator Deez = Test.begin(); Deez != Test.end(); ++Deez) {
				robot1.goTo(odom1 , (*Deez));
			}
		}
		// drive11();
		
	pros::delay(20);
	}
}




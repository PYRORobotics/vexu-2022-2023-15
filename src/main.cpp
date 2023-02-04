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

odom odom1;

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

void odomTask(){
    while(true){
        odom1.updateOdom();
        pros::delay(5);
    }
}

int rpm_target = 0;

struct TBHValues {
double error = 0;
double current = 0;
double last_error = 0;
double target = rpm_target / 3600.0;
double drive = 0;
double Kp = .02;
double drive_approx = .80; //78
int first_cross = 1;
double drive_at = 0;
double drive_at_zero = 0;
pros::Motor motor;
};

bool sgn(double num) {
    if(num > 0)
        return true;
    else
        return false;
}

void doTBH(struct TBHValues* v){
    v->target = rpm_target / 3600.0;

    v->current = v->motor.get_actual_velocity() / 200.0;


    // calculate error in velocity
    // target is desired velocity
    // current is measured velocity
    v->error = v->target - v->current;

    // Use Kp as gain
    v->drive = v->drive + (v->error * v->Kp);

    // Clip - we are only going forwards
    if(v->drive > 1 )
        v->drive = 1;
    if(v->drive < 0 )
        v->drive = 0;

    // Check for zero crossing
    if(sgn(v->error) != sgn(v->last_error) ) {
        // First zero crossing after a new set velocity command
        if( v->first_cross ) {
            // Set drive to the open loop approximation
            v->drive = v->drive_approx;
            v->first_cross = 0;
        }
        else
            v->drive = 0.5 * ( v->drive + v->drive_at_zero );

        // Save this drive value in the "tbh" variable
        v->drive_at_zero = v->drive;
    }

    // Save last error
    v->last_error = v->error;

    v->motor.move(127 * v->drive);
}

void flywheelTask(){
    struct TBHValues bigWheel = {
            .error = 0,
            .current = 0,
            .last_error = 0,
            .target = rpm_target / 3600.0,
            .drive = 0,
            .Kp = .02,
            .drive_approx = .80, //78
            .first_cross = 1,
            .drive_at = 0,
            .drive_at_zero = 0,
            .motor = flywheelBig
    };
    struct TBHValues smallWheel = {
            .error = 0,
            .current = 0,
            .last_error = 0,
            .target = rpm_target / 3600.0,
            .drive = 0,
            .Kp = .02,
            .drive_approx = .80, //78
            .first_cross = 1,
            .drive_at = 0,
            .drive_at_zero = 0,
            .motor = flywheelSmall
    };
    while(true){
        pros::lcd::print(0, "Small V: %f, T: %f", smallWheel.current * 3600, smallWheel.target * 3600);
        pros::lcd::print(1, "Big V: %f, T: %f", smallWheel.current * 3600, smallWheel.target * 3600);
        doTBH(&bigWheel);
        doTBH(&smallWheel);
        pros::delay(10);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    printf("entered initialize.\n");
		pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

    pros::delay(100);
    up.reset();
    up2.reset();
    sideways.reset();
    imu1.initialize();
    pros::delay(100);
    odom1 = odom();

    //pros::Task myTask(odomTask);
    pros::Task myFlywheelTask(flywheelTask);
    piston.set_value(false);
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

    robot1.goToCharles(&odom1, robotPose(2_ft,4_ft, 0_deg), 1_in);
    robot1.goToCharles(&odom1, robotPose(4_ft,4_ft, 90_deg), 1_in);
    robot1.goToCharles(&odom1, robotPose(2_ft,4_ft, 0_deg), 1_in);
    robot1.goToCharles(&odom1, robotPose(0_ft,0_ft, 0_deg), 1_in);



    //pros::delay(10000);
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
	//pros::delay(100);
	//up.reset();
	//sideways.reset();
	std::vector<robotPose> Test = Path::qbezierManualHeading(robotPose(Cartesian(0.0,0.0) , okapi::QAngle(0.0)) , robotPose(Cartesian(12.0, 120.0) , okapi::QAngle(180.0)) , robotPose(Cartesian(24.0,0.0) , okapi::QAngle(180.0)) , 190);
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
	//odom odom1 = odom();
	double drivePow=0;
	//pros::Motor intake ();
	while (true) {
		//pros::lcd::print(0, "AMT21_left value: %d", amt21_left.get_value());
        pros::lcd::print(2, "AMT21_right value: %d", up.get_value());
        pros::lcd::print(3, "AMT21_left value: %d", up2.get_value());
        pros::lcd::print(4, "AMT21_middle value: %d", sideways.get_value());
        //pros::lcd::print(6, "Response time: %ld micros", pros::micros() - start);
        pros::lcd::print(6, "heading_inertial: %f", imu2.get_heading());
        pros::lcd::print(7, "heading_navx: %f", imu1.get_heading());
		//odom1.updateOdom();
		int x = master.get_analog(ANALOG_LEFT_X);
		int y = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		stick1.updatePoint(x*1_in, y*1_in);
		okapi::QAngle headingInput = stick1.getHeading() + (imu1.get_heading() * 1_deg);
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
            printf("up2: %d\n", up2.get_value());
            printf("average: %f\n", (up.get_value() + up2.get_value())/2.0);
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
            //robot1.
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
			odom1.resetOdom();
		}
//		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
//			//intake.move(12);
//			for(std::vector<robotPose>::iterator Deez = Test.begin(); Deez != Test.end(); ++Deez) {
//				robot1.goTo(odom1 , (*Deez), 8);
//			}
//		}
//		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
//			robot1.goTo(odom1 , Test.at(130), 8);
//		}
		// drive11();

        /*if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            autonomous();
        }*/

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            //rpm_target += 50;
            rpm_target = 3000;
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            //rpm_target -= 50;
            rpm_target = 0;
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            piston.set_value(true);
        }
        else{
            piston.set_value(false);
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.move_velocity(200);
        }
        else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move_velocity(-200);
        }
        else{
            intake.move_velocity(0);
        }
		
	pros::delay(20);
	}
}
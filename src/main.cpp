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
#include "flywheel.h"

using okapi::inch;

odom odom1;
const Cartesian GOAL_POS(122.22, 122.22);

int calcRPMForDistance(okapi::QLength dist){
    double x = dist.convert(okapi::inch);
    return 2014 + 7.55*x + 0.0222*(x*x);
}

int calcTimeMSForDistance(okapi::QLength dist){
    double x = dist.convert(okapi::inch);
    return 1000*(00474 + 0.0146*x - 4.98E-5 * (x*x));
}

okapi::QAngle calcAngleForRPM(int x){
    return 1_deg * (41.0 - 0.0292 * x + 6.1E-6 * (x*x));
}


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

void odomTask(){
    while(true){
        odom1.updateOdom();
        pros::delay(5);
    }
}

int rpm_target = 0;

bool sgn(double num) {
    if(num > 0)
        return true;
    else
        return false;
}

void flywheelTask() {
    Flywheel bigFlywheel(13); //placeholder port
    Flywheel smallFlywheel(14); //placeholder port

    while(true) {
        bigFlywheel.set_target(rpm_target/3600.0);
        bigFlywheel.doTBH();
        smallFlywheel.set_target(rpm_target/3600.0);
        smallFlywheel.doTBH();
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

    odom1.position = Cartesian(88.5_in, 7.5_in);

    pros::Task myTask(odomTask);
    pros::Task myFlywheelTask(flywheelTask);
    indexer.set_value(false);

    sylib::initialize();
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

okapi::QAngle getAngleToPoint(Cartesian startPos, Cartesian targetPos){
    Cartesian relativePos(targetPos.x - startPos.x, targetPos.y - startPos.y);
    return relativePos.getHeading();
}

okapi::QLength getDistanceBetweenPoints(Cartesian startPos, Cartesian targetPos){
    Cartesian relativePos(targetPos.x - startPos.x, targetPos.y - startPos.y);
    return relativePos.getMagnitude();
}

void facePoint(RobotControl* robot, Cartesian pointToFace, double threshold = 0.5){
    okapi::QAngle absoluteAngle = getAngleToPoint(odom1.position, pointToFace);
    okapi::QAngle deltaAngle = absoluteAngle + 180_deg - (imu1.get_heading() * 1_deg);
    okapi::QAngle derivativeAngle = 0_deg;
    okapi::QAngle lastAngle = deltaAngle;
    printf("dA= %f\n", deltaAngle.convert(okapi::degree));
    printf("entering loop\n");
    pros::delay(10);
    double kP = 11.0;
    double kD = 9.0;
    int counter = 0;
    while((abs(deltaAngle.convert(okapi::degree)) > threshold || abs(derivativeAngle.convert(okapi::degree)) > 0.01) && !(master.get_digital(pros::E_CONTROLLER_DIGITAL_B))){
        absoluteAngle = getAngleToPoint(odom1.position, pointToFace);
        deltaAngle = absoluteAngle + 180_deg - (imu1.get_heading() * 1_deg);
        if(deltaAngle > 180_deg)
            deltaAngle -= 180_deg;

        derivativeAngle = lastAngle - deltaAngle;

        lastAngle = deltaAngle;
        robot->raw_tank(0, (deltaAngle.convert(okapi::degree) * kP) - (derivativeAngle.convert(okapi::degree) * kD), 0);

        counter++;
        if(counter%50 == 0){
            printf("dA = %f\n", deltaAngle.convert(okapi::degree));
        }

        pros::delay(20);

    }
    printf("finished loop\n");
    robot->raw_tank(0, 0, 0);
}

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
    if(true) { //true for match auto, false for skills auto

        robot1.goToPoint(&odom1, robotPose(88.5_in, 13_in, 180_deg), 1_in);
        robot1.goToPoint(&odom1, robotPose(112_in, 12_in, 180_deg), 1_in);
        robot1.goToPoint(&odom1, robotPose(112_in, 8.5_in, 180_deg), 1_in);

        pros::delay(500);
        intake.move_relative(-120, 300);
        pros::delay(2000);

        robot1.goToPoint(&odom1, robotPose(106_in, 16_in, 180_deg), 1_in);

        facePoint(&robot1, Cartesian(122.22, 122.22));

        rpm_target = 3150;
        pros::delay(5000);
        indexer.set_value(true); //shoot first disc
        pros::delay(1000);
        indexer.set_value(false);
        pros::delay(1500);

        robot1.goToPoint(&odom1, robotPose(106_in, 10_in, 180_deg), 1_in);
        robot1.goToPoint(&odom1, robotPose(106_in, 16_in, 180_deg), 1_in);

        facePoint(&robot1, Cartesian(122.22, 122.22));
        pros::delay(1000);

        indexer.set_value(true); //shoot second disc
        pros::delay(1500);
        indexer.set_value(false);
        pros::delay(1500);

        rpm_target = 0;
    }
    else{
        robot1.goToPoint(&odom1, robotPose(88.5_in, 13_in, 180_deg), 1_in);
        robot1.goToPoint(&odom1, robotPose(111_in, 13_in, 180_deg), 1_in);
        robot1.goToPoint(&odom1, robotPose(112_in, 8.5_in, 180_deg), 1_in);

        pros::delay(500);
        intake.move_relative(180, 200);
        pros::delay(2000);
        robot1.goToPoint(&odom1, robotPose(125.0_in, 30.5_in, 90_deg), 1_in);

        robot1.goToPoint(&odom1, robotPose(136.0_in, 31.5_in, 90_deg), 1.5_in);

        pros::delay(500);
        intake.move_relative(180, 200);
        pros::delay(2000);

        robot1.goToPoint(&odom1, robotPose(120_in, 24_in, 180_deg), 1_in);
        facePoint(&robot1, Cartesian(0, 144));

        pros::delay(10000);
        endgame.set_value(true);          //shoot endgame
        pros::delay(5000);
    }
    rpm_target = 0;
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

void opcontrol2(){
// Create an addrled object
    auto addrled = sylib::Addrled(22, 2, 26);
 
    // Set the LED strip to a gradient in HSV color space
    // that displays a full range of hues
    //addrled.gradient(0xFF0000, 0xFFEE00, 4);
 
    // Cycle the colors at speed 10
    //addrled.cycle(*addrled, 10);

    for(int i =0; i < 26; i++){
        addrled.set_pixel(0XFF0000 + i * 0x0600, i);
    }

    
    
    // Store the time at the start of the loop
    std::uint32_t clock = sylib::millis();
    while (true) {
        //printf("looping\n");
        // 10ms delay to allow other tasks to run
        //addrled.pulse(0x0000FF, 4, 52);
        sylib::delay_until(&clock, 500);
    }
}

float diagnosticTimer;
void opcontrol() {
	std::vector<robotPose> Test = Path::qbezierManualHeading(robotPose(Cartesian(0.0,0.0) , okapi::QAngle(0.0)) , robotPose(Cartesian(12.0, 120.0) , okapi::QAngle(180.0)) , robotPose(Cartesian(24.0,0.0) , okapi::QAngle(180.0)) , 190);
	int progress = 0;
	RobotControl robot1;
	while(imu1.is_calibrating()==true) {
		pros::delay(20);
	}
	pros::delay(100);
	Cartesian stick1;
    okapi::QAngle robotHeading = imu1.get_heading()*1_deg;
	double drivePow=0;
	while (true) {
        pros::lcd::print(2, "Distance: %f", getDistanceBetweenPoints(odom1.position, GOAL_POS).convert(okapi::inch));
        pros::lcd::print(3, "Delta heading: %f", ((imu1.get_heading() * 1_deg) - getAngleToPoint(odom1.position, GOAL_POS) - 180_deg).convert(okapi::degree));
        pros::lcd::print(6, "heading_inertial: %f", imu2.get_heading());
        pros::lcd::print(7, "heading_navx: %f", imu1.get_heading());
		int x = master.get_analog(ANALOG_LEFT_X);
		int y = master.get_analog(ANALOG_LEFT_Y);
		int turn = master.get_analog(ANALOG_RIGHT_X);
		stick1.updatePoint(x*1_in, y*1_in);
		okapi::QAngle headingInput = stick1.getHeading() + (imu1.get_heading() * 1_deg);
		double magn = pow((x*x+y*y), 0.5);
		if(fabs(normRightX())>5) {
			robotHeading = robotHeading+normRightX()*.06_deg;
		}
		if((fabs(normRightX())<=5)&&(fabs(robotHeading.convert(okapi::degree)-imu1.get_heading())>50)) {
			robotHeading = imu1.get_heading()*1_deg;
		}
		if((pros::millis()-diagnosticTimer)>1000) {
            diagnosticTimer = pros::millis();
			odom1.printOdom();
		}

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            okapi::QLength realDistToGoal = getDistanceBetweenPoints(odom1.position, GOAL_POS);
            int flightTimeMS = calcTimeMSForDistance(realDistToGoal);
            //900ms assumed flight time
            Cartesian deltaPos = odom1.deltaPositionNormalized();
            Cartesian deltaPosFudged = deltaPos;
            deltaPosFudged.scale(flightTimeMS);

            Cartesian currentPosFudged = Cartesian(deltaPosFudged.x + odom1.position.x, deltaPosFudged.y + odom1.position.y);
            okapi::QLength distFudged = getDistanceBetweenPoints(currentPosFudged, GOAL_POS); //this is a fudge factor to resolve the interdependency between distance and time of flight
            rpm_target = calcRPMForDistance(distFudged);
            okapi::QAngle offsetAngle = calcAngleForRPM(rpm_target);
            okapi::QAngle angle = getAngleToPoint(currentPosFudged, GOAL_POS) + offsetAngle;
            robot1.headingStrafe(stick1.getHeading(), magn, angle + 180_deg);
        }
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {}
        else {
            double turnVal = normRightX();
            if (fabs(turnVal) < 8){
                turnVal = 0;
            }
			robot1.absStrafe(headingInput, magn, turnVal);
		}
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
			odom1.resetOdom();
		}

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            rpm_target += 50;
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            rpm_target = 2900;
        }
        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            rpm_target -= 50;
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            indexer.set_value(true);
        }
        else{
            indexer.set_value(false);
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

        if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            facePoint(&robot1, GOAL_POS);
        }

        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            endgame.set_value(true);
        }
        else{
            endgame.set_value(false);
        }
		
	pros::delay(20);
	}
}
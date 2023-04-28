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
sylib::Motor* motor;
};

bool sgn(double num) {
    if(num > 0)
        return true;
    else
        return false;
}

void doTBH(struct TBHValues* v){
    v->target = rpm_target / 3600.0;

    v->current = -v->motor->get_velocity() / 200.0;


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

    v->motor->set_voltage(-12000.0 * v->drive);
    flywheelSmall.set_voltage(12000.0 * v->drive);
}

void flywheelTask(){
    struct TBHValues bigWheel = {
            .error = 0,
            .current = 0,
            .last_error = 0,
            .target = rpm_target / 3600.0,
            .drive = 0,
            .Kp = .04,
            .drive_approx = .80, //78
            .first_cross = 1,
            .drive_at = 0,
            .drive_at_zero = 0,
            .motor = &flywheelBig
    };
    struct TBHValues smallWheel = {
            .error = 0,
            .current = 0,
            .last_error = 0,
            .target = rpm_target / 3600.0,
            .drive = 0,
            .Kp = .04,
            .drive_approx = .80, //78
            .first_cross = 1,
            .drive_at = 0,
            .drive_at_zero = 0,
            .motor = &flywheelSmall
    };
    while(true){
        //pros::lcd::print(0, "Small V: %f, T: %f", smallWheel.current * 3600, smallWheel.target * 3600);
        pros::lcd::print(1, "Big V: %f, T: %f", bigWheel.current * 3600, bigWheel.target * 3600);
        doTBH(&bigWheel);
        //doTBH(&smallWheel);
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
    imu1.reset();
    pros::delay(100);
    odom1 = odom();

    odom1.position = Cartesian(86.5_in , 7.5_in);
    //odom1.position = Cartesian(0.0_in,0.0_in);
    pros::Task myTask(odomTask);
    pros::Task myFlywheelTask(flywheelTask);
//    indexer.set_value(false);

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
    //Cartesian absoluteGoalPos(122.22_in, 122.22_in);
    //Cartesian relativeGoalPos(pointToFace.x - odom1.getX_position(), pointToFace.y - odom1.getY_position());
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
        //relativeGoalPos = Cartesian(pointToFace.x - odom1.getX_position(), pointToFace.y - odom1.getY_position());
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
    double offset = 180.0;
    std::vector <robotPose> start_to_3stack = Path::cbezierManualHeading(
            robotPose(Cartesian(86.5_in , 7.5_in), okapi::QAngle(0.0 + offset)),
            robotPose (Cartesian(  86.5_in, 12_in ), okapi::QAngle(0.0 + offset)),
            robotPose (Cartesian(  120_in, 10_in ), okapi::QAngle(0.0 + offset)),
            robotPose(Cartesian(107.0_in , 36_in), okapi::QAngle(0.0 + offset)),
            20);
    std::vector <robotPose> stack3_to_start3Pickup = Path::qbezierManualHeading(
            robotPose(Cartesian(107.0_in , 36.0_in),
                      okapi::QAngle(0.0 + offset)),
            robotPose(Cartesian(100.0_in, 30.0_in),
                      okapi::QAngle(340.0 + offset)),
            robotPose(Cartesian(95.0_in, 26.0_in),
                      okapi::QAngle(320.0 + offset)),
            35);
    std::vector <robotPose> start3Pickup_to_end3Pickup = Path::generateStraightPath(
            robotPose(Cartesian(95.0_in, 26.0_in),
                      okapi::QAngle(320.0 + offset)),
            robotPose(Cartesian(63.0_in , 49.0_in),
                      okapi::QAngle(320.0 + offset)),
            25);
    std::vector <robotPose> stack3_to_Roller = Path::generateStraightPath(
            robotPose(Cartesian(107.0_in , 36.0_in),
                      okapi::QAngle(0.0 + offset)),
            robotPose(Cartesian(108.0_in, 7.5_in),
                      okapi::QAngle(180.0 + offset)),
            20);
    std::vector <robotPose> Roller_to_start3Pickup = Path::generateStraightPath(
            robotPose(Cartesian(108.0_in , 7.5_in),
                      okapi::QAngle(0.0 + offset)),
            robotPose(Cartesian(95.0_in, 26.0_in),
                      okapi::QAngle(330.0 + offset)),
            20);
	RobotControl robot1;
	imu1.reset();
	while(imu1.is_calibrating()) {
		pros::delay(20);
	}
	pros::delay(100);
    if(true) { //true for match auto, false for skills auto

        robot1.followCurve(&odom1, start_to_3stack, 5_in);

        robot1.followCurve( &odom1, stack3_to_start3Pickup, 5_in);

        robot1.followCurve(&odom1, start3Pickup_to_end3Pickup, 5_in);

        //robot1.raw_tank(0.0,0.0,0.0);
//        robot1.goToCharles(&odom1, robotPose(88.5_in, 13_in, 180_deg), 1_in);
//        robot1.goToCharles(&odom1, robotPose(112_in, 12_in, 180_deg), 1_in);
//        robot1.goToCharles(&odom1, robotPose(112_in, 8.5_in, 180_deg), 1_in);
//
//        pros::delay(500);
//        intake.move_relative(-120, 300);
//        pros::delay(2000);
//        //while(!intake.is_stopped()){
//        //    pros::delay(10);
//        //}
//
//        robot1.goToCharles(&odom1, robotPose(106_in, 16_in, 180_deg), 1_in);
//
//        facePoint(&robot1, Cartesian(122.22, 122.22));
//
//        rpm_target = 3150;
//        pros::delay(5000);
////        indexer.set_value(true); //shoot first disc
//        pros::delay(1000);
////        indexer.set_value(false);
//        pros::delay(1500);
//
//        robot1.goToCharles(&odom1, robotPose(106_in, 10_in, 180_deg), 1_in);
//        robot1.goToCharles(&odom1, robotPose(106_in, 16_in, 180_deg), 1_in);
//
//        //pros::delay(500);
//        facePoint(&robot1, Cartesian(122.22, 122.22));
//        pros::delay(1000);
//
////        indexer.set_value(true);
////        pros::delay(1500);
////        indexer.set_value(false);
////        pros::delay(1500);
//
//        rpm_target = 0;
    } else{
        robot1.goToCharles(&odom1, robotPose(88.5_in, 13_in, 180_deg), 1_in);
        robot1.goToCharles(&odom1, robotPose(111_in, 13_in, 180_deg), 1_in);
        robot1.goToCharles(&odom1, robotPose(112_in, 8.5_in, 180_deg), 1_in);

        pros::delay(500);
        intake.move_relative(180, 200);
        pros::delay(2000);
        //while(!intake.is_stopped()){
        //    pros::delay(10);
        //}
        robot1.goToCharles(&odom1, robotPose(125.0_in, 30.5_in, 90_deg), 1_in);

        robot1.goToCharles(&odom1, robotPose(136.0_in, 31.5_in, 90_deg), 1.5_in);

        pros::delay(500);
        intake.move_relative(180, 200);
        pros::delay(2000);

        /*robot1.goToCharles(&odom1, robotPose(122.0_in, 80_in, 180_deg), 1_in);
        facePoint(&robot1, Cartesian(122.22, 122.22));

        rpm_target = 3000;
        pros::delay(5000);
        indexer.set_value(true); //shoot first disc
        pros::delay(1000);
        indexer.set_value(false);
        pros::delay(1500);


        robot1.goToCharles(&odom1, robotPose(122_in, 70_in, 180_deg), 1_in);
        robot1.goToCharles(&odom1, robotPose(122_in, 80_in, 180_deg), 1_in);

        facePoint(&robot1, Cartesian(122.22, 122.22));
        pros::delay(1000);

        //pros::delay(500);
        facePoint(&robot1, Cartesian(122.22, 122.22));
        pros::delay(1000);

        indexer.set_value(true); //shoot second disc
        pros::delay(1500);
        indexer.set_value(false);
        pros::delay(1500);
        rpm_target = 0;

*/
        robot1.goToCharles(&odom1, robotPose(120_in, 24_in, 180_deg), 1_in);
        facePoint(&robot1, Cartesian(0, 144));

        pros::delay(10000);
        endgame.set_value(true);          //shoot endgame
        pros::delay(5000);


    }


    rpm_target = 0;

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
    //pros::delay(100);
    //up.reset();
    //sideways.reset();
    int progress = 0;
    RobotControl robot1;
    //AMT21 amt21_left(19, 0x58);
    //imu1.reset();
    while (imu1.is_calibrating() == true) {
        pros::delay(20);
    }
    pros::delay(100);
    Cartesian stick1;
    //okapi::QAngle robotHeading = 0_rad;
    okapi::QAngle robotHeading = imu1.get_heading() * 1_deg;
    //odom odom1 = odom();
    double drivePow = 0;

    //pros::Motor intake ();
    while (true) {
        pros::lcd::print(2, "Distance: %f", getDistanceBetweenPoints(odom1.position, GOAL_POS).convert(okapi::inch));
        pros::lcd::print(3, "Delta heading: %f",
                         ((imu1.get_heading() * 1_deg) - getAngleToPoint(odom1.position, GOAL_POS) - 180_deg).convert(
                                 okapi::degree));
        pros::lcd::print(6, "heading_inertial: %f", imu2.get_heading());
        pros::lcd::print(7, "heading_navx: %f", imu1.get_heading());
        //odom1.updateOdom();
        int x = master.get_analog(ANALOG_LEFT_X);
        int y = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        stick1.updatePoint(x * 1_in, y * 1_in);
        okapi::QAngle headingInput = stick1.getHeading() + (imu1.get_heading() * 1_deg);
        double magn = pow((x * x + y * y), 0.5);
        if (fabs(normRightX()) > 5) {
            robotHeading = robotHeading + normRightX() * .06_deg;
        }
        if ((fabs(normRightX()) <= 5) && (fabs(robotHeading.convert(okapi::degree) - imu1.get_heading()) > 50)) {
            robotHeading = imu1.get_heading() * 1_deg;
        }
        if ((pros::millis() - diagnosticTimer) > 1000) {
            // printf("magn: %f\n", magn);
            // printf("turn:  %d\n", turn);
            // printf("controller: %f\n", normRightX());
            // printf("robotHeading: %f\n", robotHeading.convert(okapi::degree));
            // printf("-------------------------\n");
            /*printf("up: %d\n", up.get_value());
            printf("up2: %d\n", up2.get_value());
            printf("average: %f\n", (up.get_value() + up2.get_value())/2.0);
			printf("sideways: %d\n", sideways.get_value());*/
            diagnosticTimer = pros::millis();
            odom1.printOdom();
        }
        // robot1.absStrafe(headingInput, magn, turn*1);


        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            okapi::QLength realDistToGoal = getDistanceBetweenPoints(odom1.position, GOAL_POS);
            int flightTimeMS = calcTimeMSForDistance(realDistToGoal);
            //900ms assumed flight time
            Cartesian deltaPos = odom1.deltaPositionNormalized();
            Cartesian deltaPosFudged = deltaPos;
            //deltaPosFudged.scale(700);
            deltaPosFudged.scale(flightTimeMS);

            Cartesian currentPosFudged = Cartesian(deltaPosFudged.x + odom1.position.x,
                                                   deltaPosFudged.y + odom1.position.y);
            okapi::QLength distFudged = getDistanceBetweenPoints(currentPosFudged,
                                                                 GOAL_POS); //this is a fudge factor to resolve the interdependency between distance and time of flight
            //Cartesian targetPoint =
            rpm_target = calcRPMForDistance(distFudged);
            okapi::QAngle offsetAngle = calcAngleForRPM(rpm_target);
            okapi::QAngle angle = getAngleToPoint(currentPosFudged, GOAL_POS) + offsetAngle;
            Polar got_to_goal = Polar(GOAL_POS.x - odom1.position.x, GOAL_POS.y - odom1.position.y);
            robot1.headingStrafe(stick1.getHeading(), magn, angle + 180_deg);
        } else {
            double turnVal = normRightX();
            if (fabs(turnVal) < 8) {
                turnVal = 0;
            }
            robot1.relStrafe(stick1.getHeading(), magn, turnVal);
            //robot1.
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            roller = 127;
        } else {
            roller = 0;
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            tilter.set_value(true);
        } else {
            tilter.set_value(false);
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            rpm_target = 2000;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            rpm_target = 3800;
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            rpm_target = 2500;
        }


        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            endgame.set_value(true);
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake.move_velocity(200);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake.move_velocity(-200);
        } else {
            intake.move_velocity(0);
        }

        pros::delay(20);
    }
}
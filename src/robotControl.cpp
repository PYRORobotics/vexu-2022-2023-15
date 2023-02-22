#include "main.h"
#include "robotControl.h"
#include "okapi/api/units/QAngle.hpp"
using namespace okapi::literals;

void RobotControl::raw_tank(double straightPow, double turnPow, double strafePow) {
	float left_front = (straightPow + turnPow + strafePow);
	float right_front = (straightPow - turnPow - strafePow);
	float left_back= (straightPow + turnPow - strafePow);
	float right_back = (straightPow - turnPow + strafePow);
	float highest = 0;
	if(fabs(left_front)>highest) {
		highest=fabs(left_front);
	}
	if(fabs(right_front)>highest) {
		highest=fabs(right_front);
	}
	if(fabs(left_back)>highest) {
		highest=fabs(left_back);
	}
	if(fabs(right_back)>highest) {
		highest=fabs(right_back);
	}
	float scale=0;
	if(highest<=100) {
		scale=1;
	} else {
		scale = 100/highest;
	}
	left_front0_mtr = (straightPow + turnPow + strafePow)*scale;
	left_front1_mtr = (straightPow + turnPow + strafePow)*scale;
	left_back0_mtr = (straightPow + turnPow - strafePow)*scale;
	left_back1_mtr = (straightPow + turnPow - strafePow)*scale;
	right_front0_mtr = -((straightPow - turnPow - strafePow)*scale);
	right_front1_mtr = -((straightPow - turnPow - strafePow)*scale);
	right_back0_mtr = -((straightPow - turnPow + strafePow)*scale);
	right_back1_mtr = -((straightPow - turnPow + strafePow)*scale);
}

void RobotControl::relStrafe(okapi::QAngle relHeading, double pow, double turn) {
	double straight = pow*cos(relHeading.convert(okapi::radian));
	double strafe = pow*sin(relHeading.convert(okapi::radian));
	this->raw_tank(straight, turn, strafe);
}

void RobotControl::absStrafe(okapi::QAngle absHeading, double pow, double turn) {
	this->relStrafe(absHeading-imu1.get_heading()*1_deg, pow, turn);		
}
void RobotControl::headingStrafe(okapi::QAngle driveHeading, double pow, okapi::QAngle turnHeading) {
	//helps calculate the fastest way to turn to the desired heading
	double error = turnHeading.convert(okapi::degree)-imu1.get_heading()+360;
	double test = turnHeading.convert(okapi::degree)-imu1.get_heading()+360;
	error = fmod(error, 360);
	test = fmod(test, 360);

	if(error>180) {
		error=error-360;
	}
	double turnPow = error*driveHeading_kP;
	if(turnPow>80) {
		turnPow=80;
	} else if(turnPow<-80) {
		turnPow = -80;
	} 
	this->absStrafe(driveHeading, pow, turnPow);
	if((pros::millis()-timer)>1000000000000000000) {
		printf("error: %f\n", error);
		printf("RAW: %f\n",  turnHeading.convert(okapi::degree)-imu1.get_heading());
		printf("test: %f\n", test);

		timer = pros::millis();
	}
}
void RobotControl::goToPoint(odom *odom1, robotPose robotPose, okapi::QLength threshold) {
    double drivePow;
    Cartesian delta(robotPose.position.x - odom1->position.x, robotPose.position.y - odom1->position.y);
    int i = 0;
    while(delta.getMagnitude() >= (threshold) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        delta.x = (robotPose.position.x - odom1->position.x);
        delta.y = (robotPose.position.y - odom1->position.y);
        drivePow = delta.getMagnitude().convert(okapi::inch) * 30; // basic as heck P loop
        if(drivePow > 100){
            drivePow = 100;
        }

        this->headingStrafe(delta.getHeading()+0_deg, drivePow * 1.0, (robotPose.heading));
        if(i%100 == 0){
            printf("Dmag: %f\n", delta.getMagnitude().convert(okapi::inch));
            odom1->printOdom();
        }
        i++;
        pros::delay(10);
    }
}
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
	right_front0_mtr = (straightPow - turnPow - strafePow)*scale;
	right_front1_mtr = (straightPow - turnPow - strafePow)*scale;
	right_back0_mtr = (straightPow - turnPow + strafePow)*scale;
	right_back1_mtr = (straightPow - turnPow + strafePow)*scale;
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

void RobotControl::goTo(Cartesian Cartiesian) {
}
//auton p looper, goes to position given and faces the bot tword heading
void RobotControl::goTo(odom odom1, robotPose robotPose) {
	Cartesian bot_pos(odom1.getX_position(), odom1.getY_position());
	Polar to_Polar(robotPose.position.x - bot_pos.x, (robotPose.position.y - bot_pos.y));
	Cartesian to_Cartesian(to_Polar);
	double drivePow;
	double progress = 1;
	while(sqrt((to_Cartesian.x * to_Cartesian.x) + (to_Cartesian.y * to_Cartesian.y)) >= 8_in){
		drivePow = to_Polar.magnitude.convert(okapi::inch)*30;
		if (drivePow>100) {
			drivePow = 100;
		}
		this->headingStrafe(odom1.position.getHeading()+180_deg, drivePow, robotPose.heading * (1 - progress));
		progress = progress * 0.8;
	}
}
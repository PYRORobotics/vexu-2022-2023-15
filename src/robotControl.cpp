#include "main.h"
#include "../include/robotControl.h"
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
	double strafe = -(pow*sin(relHeading.convert(okapi::radian)));
	this->raw_tank(straight, turn, -strafe);// CHANGE THE ENCODERS IF CHANGING STRAFE SIGN,AKA MAKE THE LEFT ONE THE RIGHT ON AND VIS VERSA
}

void RobotControl::absStrafe(okapi::QAngle absHeading, double pow, double turn) {
	this->relStrafe(absHeading-imu1->get_heading()*1_deg, pow, turn);		
}
void RobotControl::headingStrafe(okapi::QAngle driveHeading, double pow, okapi::QAngle turnHeading) {
	//helps calculate the fastest way to turn to the desired heading
	double error = turnHeading.convert(okapi::degree)-imu1->get_heading()+360;
	double test = turnHeading.convert(okapi::degree)-imu1->get_heading()+360;
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
		printf("RAW: %f\n",  turnHeading.convert(okapi::degree)-imu1->get_heading());
		printf("test: %f\n", test);

		timer = pros::millis();
	}
}
void RobotControl::goToCharles(odom *odom1, robotPose robotPose, okapi::QLength threshold) {
    double drivePow;
    Cartesian delta(robotPose.position.x - odom1->position.x, robotPose.position.y - odom1->position.y);
    int i = 0;
    while(delta.getMagnitude() >= (threshold) && !master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        //printf("1");
        //odom1->updateOdom();
        //printf("2");
        delta.x = (robotPose.position.x - odom1->position.x);
        delta.y = (robotPose.position.y - odom1->position.y);
        drivePow = delta.getMagnitude().convert(okapi::inch) * 30; // basic as heck P loop
        if(drivePow > 100){
            drivePow = 100;
        }

        this->headingStrafe(delta.getHeading()+0_deg, drivePow * 1.0, (robotPose.heading));
        //printf("3\n");
        if(i%100 == 0){
            printf("Dmag: %f\n", delta.getMagnitude().convert(okapi::inch));
            odom1->printOdom();
            //printf("Ox: %f\n", odom1.position.x.convert(okapi::inch));
            //printf("Oy: %f\n", odom1.position.y.convert(okapi::inch));
        }
        i++;
        pros::delay(10);
    }
}
void RobotControl::goTo(odom *odom1, robotPose robotPose, okapi::QLength follow_Dist) {
    double drivePow;
    int i = 0;
    Cartesian delta(robotPose.position.x - odom1->position.x, robotPose.position.y - odom1->position.y);
    while(delta.getMagnitude() >= (follow_Dist)) {
        delta.x = (robotPose.position.x - odom1->position.x);
        delta.y = (robotPose.position.y - odom1->position.y);
        drivePow = delta.getMagnitude().convert(okapi::inch) * 30;
        if(drivePow > 100){
            drivePow = 100;
        }
        this->headingStrafe(delta.getHeading()+0_deg, drivePow * 1.0, (robotPose.heading));
        if(i%100 == 0){
            printf("Dmag: %f\n", delta.getMagnitude().convert(okapi::inch));
            printf("Angle: %f" , imu1->get_heading());
            odom1->printOdom();
            //printf("Ox: %f\n", odom1.position.x.convert(okapi::inch));
            //printf("Oy: %f\n", odom1.position.y.convert(okapi::inch));
        }
        i++;
        pros::delay(10);
    }
}
void RobotControl::followCurve(odom *odom, std::vector<robotPose> path, okapi::QLength follow_Dist) {
    for(int i = 0; i < path.size(); i++) {
        this->goTo(odom, path.at(i), follow_Dist);
    }
}
/*
//auton p looper, goes to position given and faces the bot tword heading, if you do not specify an angle the robot will face 0_rad.
void RobotControl::goTo(odom odom1, robotPose robotPose, uint8_t follow_Dist) {
	Cartesian current_pos(odom1.getX_position() , odom1.getY_position());
	Polar to_Polar(robotPose.position.x - current_pos.x, robotPose.position.y - current_pos.y);
	Cartesian to_Cartesian(to_Polar);
	double drivePow;
	double progress = 1;
	while(sqrt((to_Cartesian.x * to_Cartesian.x) + (to_Cartesian.y * to_Cartesian.y)) >= (follow_Dist * 1_in)) {
		drivePow = to_Polar.magnitude.convert(okapi::inch)*30;
		if (drivePow>100) {
			drivePow = 100;
		}
        //this->headingStrafe(to_Polar.angle+180_deg, drivePow * .3, (robotPose.heading * (1 - progress)));
        this->headingStrafe(to_Polar.angle+0_deg, drivePow * .3, (robotPose.heading * (1 - progress)));
		progress = progress * 0.8;
	}
}
//auton p looper, goes to position given and faces the bot tword heading, if you do not specify an angle the robot will face 0_rad.
void RobotControl::goTo(odom odom1, Cartesian to_Cartesian, okapi::QAngle robotFacing, uint8_t follow_Dist) {
	Cartesian current_pos(odom1.getX_position() , odom1.getY_position());
	Polar to_Polar(to_Cartesian);
	double drivePow;
	while(sqrt((to_Cartesian.x * to_Cartesian.x) + (to_Cartesian.y * to_Cartesian.y)) >= (follow_Dist * 1_in)) {
		drivePow = to_Polar.magnitude.convert(okapi::inch)*30;
		if (drivePow>100) {
			drivePow = 100;
		}
		this->headingStrafe(to_Polar.angle+180_deg, drivePow * .4 , robotFacing);
	}
}
//auton p looper, goes to position given and faces the bot tword heading, if you do not specify an angle the robot will face 0_rad.
void RobotControl::goTo(odom odom1, Cartesian to_Cartesian, uint8_t follow_Dist) {
	Cartesian current_pos(odom1.getX_position() , odom1.getY_position());
	Polar to_Polar(to_Cartesian);
	double drivePow;
	while(sqrt((to_Cartesian.x * to_Cartesian.x) + (to_Cartesian.y * to_Cartesian.y)) >= (follow_Dist * 1_in)) {
		drivePow = to_Polar.magnitude.convert(okapi::inch)*30;
		if (drivePow>100) {
			drivePow = 100;
		}
		this->headingStrafe(to_Polar.angle+180_deg, drivePow * .3 , 0_rad);
	}
}*/

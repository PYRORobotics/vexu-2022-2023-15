#include "main.h"
#include "../include/odom.h"
#include "okapi/api/units/QAngle.hpp"
#include "cartesian.h"
#include "polar.h"
//#include "okapi/api/util/logging.hpp"
using namespace okapi::literals;
odom::odom() {
    printf("Odom Initialized");
    position = Cartesian(0_in, 0_in);
    lastPosition = Cartesian(0_in, 0_in);
    previousRelativeY = up.get_value()*RLCONV;
    previousRelativeX = sideways.get_value()*YCONV;
    previousInert = imu1.get_heading()*1_deg;
}
void odom::resetOdom() {
    position = Cartesian(0_in, 0_in);
    previousRelativeY = (up.get_value())*RLCONV;
    previousRelativeX = sideways.get_value()*YCONV;
    previousInert = imu1.get_heading()*1_deg;
}
// void odom::updateOdom() {
//     odomAngle = ((imu1.get_heading()*1_rad)+previousInert)/2;
//     yRelativeDelta = (up.get_value()+up.get_value())*RLCONV-previousRelativeY;
//     xRelativeDelta = sideways.get_value()*YCONV - previousRelativeX;
//     //deal with the y-component of the relative motion
//     yPosition += cos(odomAngle.convert(okapi::radian))*yRelativeDelta;
//     xPosition += sin(odomAngle.convert(okapi::radian))*yRelativeDelta;
//     // deal with the x-component of the relative motion
//     yPosition += sin(odomAngle.convert(okapi::radian))*xRelativeDelta;
//     xPosition += -cos(odomAngle.convert(okapi:: radian))*xRelativeDelta;


//     previousRelativeY= (up.get_value()+up.get_value())*RLCONV;
//     previousRelativeX = sideways.get_value()*YCONV;
//     previousInert = imu1.get_heading()*1_rad;
// }
void::odom::updateOdom() {
    double up1_val = up.get_value();
    double up2_val = up2.get_value();
    double sideways_val = sideways.get_value();
    double heading = imu1.get_heading();

    lastPosition = position;
    lastTimestamp = currentTimestamp;

    yRelativeDelta = ((up1_val + up2_val)/2.0)*RLCONV - previousRelativeY;
    xRelativeDelta = sideways_val*YCONV - previousRelativeX;
    //robot relative
    Polar delta = Polar(xRelativeDelta,yRelativeDelta);
    //converting to field relative
    odomAngle= ((previousInert+imu1.get_heading()*1_deg)/2);
    delta.addAngle(heading*1_deg);
    //add to current position
    position.add(delta);
    previousRelativeY = ((up1_val + up2_val)/2.0)*RLCONV;
    previousRelativeX = sideways_val*YCONV;
    previousInert = heading*1_deg;

    currentTimestamp = pros::millis();

    // printf("yRelativeDelta: %lf\n", yRelativeDelta);
    // printf("xRelativeDelta: %lf\n", xRelativeDelta);
    // printf("xRelativeDelta: %lf\n", delta.magnitude);
}

void odom::printOdom() {
    printf("Xpos: %f\n", this->position.x.convert(okapi::inch));
    printf("Ypos: %f\n", this->position.y.convert(okapi::inch));
    printf("------------------------------------------------------------------------------------------------------\n");
}

okapi::QLength odom::getX_position() {
    return position.x;
}

okapi::QLength odom::getY_position() {
    return position.y;
}

long odom::getTimestamp(){
    return currentTimestamp;
}

Cartesian odom::deltaPositionNormalized(){
    long deltaTime = currentTimestamp - lastTimestamp;
    return Cartesian((position.x - lastPosition.x)/(double)deltaTime, (position.y - lastPosition.y)/(double)deltaTime);
}
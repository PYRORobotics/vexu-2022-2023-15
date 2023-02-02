#include "main.h"
#include "odom.h"
#include "okapi/api/units/QAngle.hpp"
#include "cartesian.h"
#include "polar.h"
//#include "okapi/api/util/logging.hpp"
using namespace okapi::literals;
odom::odom() {
    printf("Odom Initialized");
    xPosition = 0_in;
    yPosition = 0_in;
    position = Cartesian(0_in, 0_in);
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
    yRelativeDelta = (up.get_value())*RLCONV-previousRelativeY;
    xRelativeDelta = sideways.get_value()*YCONV - previousRelativeX;
    //robot relative
    Polar delta = Polar(xRelativeDelta,yRelativeDelta);
    //converting to field relative
    odomAngle= ((previousInert+imu1.get_heading()*1_deg)/2);
    delta.addAngle(imu1.get_heading()*1_deg);
    //add to current position
    position.add(delta);
    previousRelativeY = (up.get_value())*RLCONV;
    previousRelativeX = sideways.get_value()*YCONV;
    previousInert = imu1.get_heading()*1_deg;

    // printf("yRelativeDelta: %lf\n", yRelativeDelta);
    // printf("xRelativeDelta: %lf\n", xRelativeDelta);
    // printf("xRelativeDelta: %lf\n", delta.magnitude);
}

void odom::printOdom() {
    printf("Xpos: %f\n", this->position.x.convert(okapi::inch));
    printf("Ypos: %f\n", this->position.y.convert(okapi::inch));
    printf("------------------------------------------------------------------------------------------------------\n");
}
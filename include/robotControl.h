#ifndef robotControl_h
#define robotControl_h
#include "main.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

class RobotControl {
public:
    void raw_tank(double straightPow, double turnPow, double strafePow);

    void relStrafe(okapi::QAngle relHeading, double pow, double turn);

    void absStrafe(okapi::QAngle absHeading, double pow, double turn);

    //constexpr static const float driveHeading_kP = 2;
    constexpr static const float driveHeading_kP = 5;

    void headingStrafe(okapi::QAngle driveHeading, double pow, okapi::QAngle turnHeading);

    constexpr static const float powerMod = 0.3;
    double timer;

    void goTo(odom *odom1, robotPose robotPose, okapi::QLength follow_Dist);

    void goTo(odom odom1, Cartesian to_Cartesian, int follow_Dist);

    void goTo(odom odom1, robotPose to_robotPose, int follow_Dist);

    void goToCharles(odom *odom1, robotPose robotPose, okapi::QLength threshold);

    void followCurve(odom *odom, std::vector<robotPose> path, okapi::QLength follow_Dist);
};
#endif
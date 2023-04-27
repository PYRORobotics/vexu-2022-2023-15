#ifndef odom_h
#define odom_h
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "cartesian.h"
#include "polar.h"
#include "robotPose.h"
class Cartesian;

class odom{
    private:
    #define RLCONV 0.00038328_in
    #define YCONV 0.00038328_in
    okapi::QLength rightConversion;
    okapi::QLength xRelativeDelta;
    okapi::QLength yRelativeDelta;
    okapi::QLength previousRelativeX;
    okapi::QLength previousRelativeY;
    okapi::QAngle odomAngle;
    okapi::QAngle previousInert;


    public:
    odom();
    Cartesian position;
    Cartesian lastPosition;
    long lastTimestamp = 0;
    long currentTimestamp = 0;
    void resetOdom();
    void updateOdom();
    okapi::QLength getX_position();
    okapi::QLength getY_position();
    double getHeading_encoders(double radius);
    void printOdom();
    long getTimestamp();

    Cartesian deltaPositionNormalized();


};

#endif
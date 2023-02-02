#ifndef robotPose_h
#define robotPose_h
#include "main.h"
#include "cartesian.h"
#include "polar.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

class Cartesian;
class Polar;
//Refrence angle is +Y
class robotPose {
    public:
    robotPose(Cartesian position, okapi::QAngle heading);
    robotPose(Polar position, okapi::QAngle heading);
    robotPose(okapi::QLength xPosition,okapi::QLength yPosition, okapi::QAngle heading);
    robotPose();
    okapi::QAngle heading;
    Cartesian position;
};

#endif
#include "main.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "polar.h"
#include "cartesian.h"
#include "robotPose.h"

robotPose::robotPose(Cartesian position, okapi::QAngle heading) {
    this->position = position;
    this->heading = heading;
}
robotPose::robotPose(Polar position, okapi::QAngle heading) {
    this->position = Cartesian(position);
    this->heading = heading;
}
robotPose::robotPose(okapi::QLength xPosition,okapi::QLength yPosition, okapi::QAngle heading){
    this->position = Cartesian(xPosition, yPosition);
    this->heading = heading;
}



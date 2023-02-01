#ifndef cartesian_h
#define cartesian_h
#include "main.h"
#include "polar.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
class Polar;
class Cartesian {
    public:
    Cartesian();
    Cartesian(okapi::QLength x, okapi::QLength y);
    Cartesian(Polar point);
    Cartesian(Cartesian const &point);
    //ASSUMES INCHES
    Cartesian(double x, double y);
    void add(Cartesian point2);
    void add(Polar point2);
    void scale(double scale);
    void updatePoint(okapi::QLength x, okapi::QLength y);
    void updatePoint(Polar point);
    okapi::QAngle getHeading();
    okapi::QLength getMagnitude();


    okapi::QLength x;
    okapi::QLength y;
};




#endif
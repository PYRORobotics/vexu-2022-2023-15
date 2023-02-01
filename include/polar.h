#ifndef polar_h
#define polar_h
#include "main.h"
#include "cartesian.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"

class Cartesian;
//Refrence angle is +Y
class Polar {
    private:
    
    public:
    Polar();
    Polar(okapi::QLength magn, okapi::QAngle angle);
    Polar(Cartesian point);
    Polar(okapi::QLength x, okapi::QLength y);
    void upDatePoint(okapi::QLength magn, okapi::QAngle angle);
    void upDatePoint(Cartesian point);
    void addAngle(okapi::QAngle add);
    void scaleMagnitude(double scale);
    okapi::QAngle angle;
    okapi::QLength magnitude;
};
#endif
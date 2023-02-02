#include "main.h"
#include "okapi/api/units/QLength.hpp"
#include "polar.h"
#include "cartesian.h"
//CONSTRUCTORS-------------------------------------------------------------------------------------------------------------------------
Polar::Polar() {
    this->angle =0_deg;
    this->magnitude = 0_in;
}
Polar::Polar(okapi::QLength magn, okapi::QAngle angle) {
    this->angle=angle;
    this->magnitude = magn;
}

Polar::Polar(Cartesian point) {
    magnitude = pow(pow(point.x.convert(okapi::inch),2)+pow(point.y.convert(okapi::inch),2),0.5)*1_in;
    if(point.x==0_in&&point.y==0_in) {
        angle = 0_deg;
    } else {
        angle = atan(point.x.convert(okapi::inch)/point.y.convert(okapi::inch))*1_rad;
    }
    if (point.y<0_in) {
        angle += 180_deg;
    }
}
Polar::Polar(okapi::QLength x, okapi::QLength y) {
    magnitude = pow(pow(x.convert(okapi::inch),2)+pow(y.convert(okapi::inch),2),0.5)*1_in;
    if(x==0_in&&y==0_in) {
        angle = 0_deg;
    } else {
        angle = atan(x.convert(okapi::inch)/y.convert(okapi::inch))*1_rad;
    }
    if (y<0_in) {
        angle += 180_deg;
    }
}



//Modifier Methods---------------------------------------------------------------------------------------------------------------------
void Polar::upDatePoint(okapi::QLength magn, okapi::QAngle angle) {
    this->angle=angle;
    this->magnitude = magn;
}

void Polar::addAngle(okapi::QAngle add) {
    angle = angle + add;
}
void Polar::upDatePoint(Cartesian point) {
    magnitude = pow(pow(point.x.convert(okapi::inch),2)+pow(point.y.convert(okapi::inch),2),0.5)*1_in;
    if(point.x==0_in&&point.y==0_in) {
        angle = 0_deg;
    } else {
        angle = atan(point.x.convert(okapi::inch)/point.y.convert(okapi::inch))*1_rad;
    }
    if (point.y<0_in) {
        angle += 180_deg;
    }
}


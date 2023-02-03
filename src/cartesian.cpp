#include "main.h"
#include "cartesian.h"
#include "okapi/api/units/QAngle.hpp"
#include "polar.h"
//CONSTRUCTORS-------------------------------------------------------------------------------------------------------------------------
Cartesian::Cartesian() {
    this->x = 0_in;
    this->y = 0_in;
}

Cartesian::Cartesian(okapi::QLength x, okapi::QLength y) {
    this->x=x;
    this->y=y;
}
Cartesian::Cartesian(Polar point) {
    x = point.magnitude*sin(point.angle.convert(okapi::radian));
    y = point.magnitude*cos(point.angle.convert(okapi::radian));
}
//ASSUMES INCHES 
Cartesian::Cartesian(double x, double y) {
    this->x=x*1_in;
    this->y=y*1_in;
}
//copy constructor

Cartesian::Cartesian(Cartesian const &point) {
    this->x = point.x;
    this->y = point.y;
}

//Getter Methods---------------------------------------------------------------------------------------------------------------------
okapi::QAngle Cartesian::getHeading() {
    okapi::QAngle angle = 0_deg;
    if(x==0_in&&y==0_in) {
        angle = 0_deg;
    } else {
        angle = atan(x.convert(okapi::inch)/y.convert(okapi::inch))*1_rad;
    }
    if (y<0_in) {
        angle += 180_deg;
    }
    return angle;
}
okapi::QLength Cartesian::getMagnitude(){
    return pow(pow(x.convert(okapi::inch),2)+pow(y.convert(okapi::inch),2),0.5)*1_in;
}


//Modifier Methods---------------------------------------------------------------------------------------------------------------------
void Cartesian::updatePoint(okapi::QLength x, okapi::QLength y) {
    this->x=x;
    this->y=y;
}

void Cartesian::updatePoint(Polar point) {
    x = point.magnitude*sin(point.angle.convert(okapi::radian));
    y = point.magnitude*cos(point.angle.convert(okapi::radian));
}

void Cartesian::add(Polar point2) {
    Cartesian convert = Cartesian(point2);
    this->add(convert);
}
void Cartesian::add(Cartesian point2) {
    this->x = this->x + point2.x;
    this->y = this->y + point2.y;
}
void Cartesian::scale(double scale) {
    this->x = this->x*scale;
    this->y = this->y*scale;
}

#include "navX.h"

navX::navX(int port) : arduino(port) {

}

void navX::initialize() {
    arduino.initialize();
    pros::delay(100);
    reset();
}

double navX::get_heading(bool doOffset) {
    int rawValue = arduino.getCounter();
    double scaledValue = rawValue/100.0;
    if(doOffset){
        scaledValue += offset;
        scaledValue += 180; //robot starts facing backwards
    }

    double convertedValue = scaledValue;
    while (convertedValue < 0)
        convertedValue += 360;

    while (convertedValue > 360)
        convertedValue -= 360;
    //(scaledValue < 0 ? scaledValue + 360 : scaledValue); //convert from [-180, 180) to [0, 360)
    return convertedValue;
}

void navX::reset(){
    offset = -get_heading(false);
}

bool navX::is_calibrating() {
    return false;
}
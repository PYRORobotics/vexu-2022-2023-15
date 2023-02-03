#include "navX.h"

navX::navX(int port) : arduino(port) {

}

void navX::initialize() {
    arduino.initialize();
}

double navX::get_heading_raw() {
    int rawValue = arduino.getCounter();
    double scaledValue = rawValue/100.0;
    double convertedValue = (scaledValue < 0 ? scaledValue + 360 : scaledValue); //convert from [-180, 180) to [0, 360)
    return convertedValue;
}

double navX::get_heading() {
    return get_heading_raw() + offset;
}

void navX::reset(){
    offset = -get_heading_raw();
}

bool navX::is_calibrating() {
    return false;
}
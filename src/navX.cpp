#include "navX.h"

navX::navX(int port) : arduino(port) {

}

double navX::getHeading() {
    int rawValue = arduino.getCounter();
    return rawValue/100.0;
}
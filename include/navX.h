#include "main.h"
#include "../src/arduino-comms.h"

class navX{
    public:
        navX(int port);
        
        double getHeading();

    private:
        Arduino arduino;
};
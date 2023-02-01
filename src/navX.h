#include "main.h"
#include "arduino-comms.h"

class navX{
    public:
        navX(int port);
        
        double getHeading();

    private:
        Arduino arduino;
};
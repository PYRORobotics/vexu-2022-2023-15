//#include "main.h"
#include "../src/arduino-comms.h"

class navX{
    public:
        navX(int port);
        
        double get_heading();
        bool is_calibrating();
        void reset();

    private:
        Arduino arduino;
        double get_heading_raw();
        double offset;

};
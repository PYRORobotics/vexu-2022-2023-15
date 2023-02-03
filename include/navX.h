//#include "main.h"
#include "../src/arduino-comms.h"

class navX{
    public:
        navX(int port);
        
        double get_heading(bool doOffset = true);
        bool is_calibrating();
        void reset();
        void initialize();

    private:
        Arduino arduino;
        double offset;

};
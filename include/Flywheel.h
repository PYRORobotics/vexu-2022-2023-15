#include "main.h"
#include "../src/arduino-comms.h"

class Flywheel {
    private:
        double error = 0;
        double current = 0;
        double last_error = 0;
        double target = 0; //rpm_target / 3600.0
        double drive = 0;
        double Kp = .02;
        double drive_approx = .80; //78
        int first_cross = 1;
        double drive_at = 0;
        double drive_at_zero = 0;
        sylib::Motor* motor;
    
    public:
        Flywheel(int port);
        void set_target(double rpm_target);
        double get_target();
        void doTBH();
        bool sgn(double num);
};
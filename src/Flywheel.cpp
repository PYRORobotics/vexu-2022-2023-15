#include "flywheel.h"

flywheel::flywheel(int port) {

}

void flywheel::set_target(double rpm_target) { target = rpm_target / 3600.0; }

double flywheel::get_target() { return target; }

bool flywheel::sgn(double num) {
    if(num > 0)
        return true;
    else
        return false;
}

void flywheel::doTBH() {

    current = motor->get_velocity() / 200.0;


    // calculate error in velocity
    // target is desired velocity
    // current is measured velocity
    error = target - current;

    // Use Kp as gain
    drive = drive + (error * Kp);

    // Clip - we are only going forwards
    if(drive > 1)
        drive = 1;
    if(drive < 0)
        drive = 0;

    // Check for zero crossing
    if(sgn(error) != sgn(last_error)) {
        // First zero crossing after a new set velocity command
        if(first_cross) {
            // Set drive to the open loop approximation
            drive = drive_approx;
            first_cross = 0;
        }
        else
            drive = 0.5 * (drive + drive_at_zero);

        // Save this drive value in the "tbh" variable
        drive_at_zero = drive;
    }

    // Save last error
    last_error = error;

    motor->set_voltage(12000.0 * drive);
}
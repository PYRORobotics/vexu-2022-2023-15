//
// Created by charles on 9/13/22.
//

#ifndef VEXU_2022_2023_TESTBED_ARDUINO_COMMS_H
#define VEXU_2022_2023_TESTBED_ARDUINO_COMMS_H
//#include "main.h"
#include "pros/apix.h"
#include "proto/messages.pb.h"
#include "cobs/cobs.h"
class Arduino{
public:
    explicit Arduino(int port);
    void setD1(bool value);

    void setD2(bool value);

    bool getD1();

    bool getD2();

    int32_t getCounter();
private:
    pros::Serial serial_port;
    pros::MutexVar<Status> status;
    pros::MutexVar<Command> command;
    void get_status();


    void log_msg(const char *format, ...);

    void write_command();

    void comms_thread();

    void error_msg(const char *format, ...);

    void debug_msg(const char *format, ...);
};
#endif //VEXU_2022_2023_TESTBED_ARDUINO_COMMS_H

//
// Created by charles on 10/4/22.
//

#ifndef VEXU_2022_2023_TESTBED_AMT21_H
#define VEXU_2022_2023_TESTBED_AMT21_H
#include "pros/apix.h"

//make this an enum?
#define AMT21_GET_POSITION_COMMAND 0x0
#define AMT21_GET_ROTATIONS_COMMAND 0x1
#define AMT21_EXTENDED_COMMAND 0x2
#define AMT21_EXTENDED_SET_ZERO 0x5E
#define AMT21_EXTENDED_RESET 0x75

#define AMT21_INVALID (30000)


class AMT21 {
public:
    AMT21(int port, uint8_t address, bool reversed = false);

    uint16_t get_position();
    uint16_t get_position_safe();

    int16_t get_turns();
    int16_t get_turns_safe();

    long get_value(bool withOffset = true);
    void reset();

private:
    bool validate_response(uint16_t value);
    uint8_t address;
    pros::Serial serial_port;
    long offset = 0;
    long last_value = 0;
    bool reversed;

};


#endif //VEXU_2022_2023_TESTBED_AMT21_H

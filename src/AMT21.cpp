//
// Created by charles on 10/4/22.
//

#include "AMT21.h"

#define DELAY_TIME 1000

AMT21::AMT21(int port, uint8_t address, bool reversed) : serial_port(port, 115200), address(address), reversed(reversed){
    //default baud for our model is 115200
    //serial_port.set_baudrate(115200);

    //ensure the address is valid (that is, a multiple of 4)
    if(address % 4 !=0){
        //TODO: throw an exception here
    }
}

uint16_t AMT21::get_position() {
    serial_port.write_byte(this->address | AMT21_GET_POSITION_COMMAND);
    //pros::delay(2);
    uint64_t startTime = pros::micros();
    while(pros::micros() -startTime < DELAY_TIME);
    uint8_t buf[2];
    while(serial_port.get_read_avail() > 2){
        serial_port.read(buf, 1);
    }
    serial_port.read( buf, 2);
    uint16_t value = (buf[0]) + ((buf[1] & 0b00111111) << 8);
    if(serial_port.get_read_avail() > 0){
        printf("Position Bytes left over: %d\n", serial_port.get_read_avail());
        serial_port.flush();
        return AMT21_INVALID;
    }

    //return validate_response(value) ? value : AMT21_INVALID;
    return value;
}

uint16_t AMT21::get_position_safe() {
    uint64_t startTime = pros::micros();
    while(pros::micros() -startTime < 280);
    return this->get_position();
}

int16_t AMT21::get_turns() {
    serial_port.write_byte(this->address | AMT21_GET_ROTATIONS_COMMAND);
    //pros::delay(2);
    uint64_t startTime = pros::micros();
    while(pros::micros() -startTime < DELAY_TIME);
    uint8_t buf[2];
    while(serial_port.get_read_avail() > 2){
        serial_port.read(buf, 1);
    }
    serial_port.read(buf, 2);
    int16_t value = (int16_t)(((buf[0]) + ((buf[1] & 0b00111111) << 8)) << 2)/4;

    if(serial_port.get_read_avail() > 0){
        printf("Turns Bytes left over: %d\n", serial_port.get_read_avail());
        serial_port.flush();
        return AMT21_INVALID;
    }

    //return validate_response(value) ? value : AMT21_INVALID;
    return value;
}

int16_t AMT21::get_turns_safe() {
    uint64_t startTime = pros::micros();
    while(pros::micros() -startTime < 280);
    return this->get_turns();
}

bool AMT21::validate_response(uint16_t value) {
    uint8_t parity = value >> 14;
    uint8_t calculated_parity = 0;

    //loop over all 14 data bits
    for(int i = 0; i < 14; i++){
        calculated_parity ^= (value & 1) << (i%2 != 0);
        value = value >> 1;
    }
    return parity == calculated_parity;
}

long AMT21::get_value(bool withOffset){
    int turns = get_turns();
    int position = get_position();

    if(turns == AMT21_INVALID || position == AMT21_INVALID){
        return last_value;
    }
    last_value = (turns * 16384) + position + (withOffset ? offset : 0);

    return last_value * ((reversed && withOffset) ? -1 : 1);
}

void AMT21::reset(){
    offset = -get_value(false);
    printf("new offset value: %d\n", offset);
}






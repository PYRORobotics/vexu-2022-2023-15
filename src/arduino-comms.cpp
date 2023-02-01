//
// Created by charles on 9/13/22.
//

#include <sstream>
#include "arduino-comms.h"
//#define DEBUG
//#define LOG
//#define ERROR

Arduino::Arduino(int port) : serial_port(port, 115200){
    pros::Task comms([this] { this->comms_thread(); });
}

void Arduino::debug_msg(const char *format, ...){
#ifdef DEBUG
    printf("[DEBUG] Arduino (%d): ", this->serial_port.get_port());
    va_list args;
    va_start(args, format);
    printf(format, args);
    va_end(args);
#endif
}

void Arduino::log_msg(const char *format, ...){
#ifdef LOG
    printf("[LOG] Arduino (%d): ", this->serial_port.get_port());
    va_list args;
    va_start(args, format);
    printf(format, args);
    va_end(args);
#endif
}

void Arduino::error_msg(const char *format, ...){
#ifdef ERROR
    printf("<<ERROR>> Arduino (%d): ", this->serial_port.get_port());
    va_list args;
    va_start(args, format);
    printf(format, args);
    va_end(args);
#endif
}

void Arduino::get_status(){
    char buf[255];
    log_msg("waiting for message\n");
    int i = 0;
    for(; i < 255; i++){
        //printf("waiting for byte\n");
        while(!serial_port.get_read_avail()){
            //printf("waiting for byte\n");
            pros::delay(1);
        }

        buf[i] = serial_port.read_byte();
        debug_msg("got byte %02X\n", buf[i]);
        //printf("received a byte: %d\n", i);
        if (buf[i] == '\0'){
            //i++;
            break;
        }
    }
    if(i == 255){
        error_msg("Packet len exceeded buffer size");
        return;
    }

    //printf("Timestamp: %ld\n", pros::millis());
    std::ostringstream foo;
    
    foo << "Received " << std::hex << (i) << " COBS-encoded bytes: ";
    //printf("Received %d COBS-encoded bytes: ", i);
    for(int j = 0; j < i; j++){
        if (j > 0)
            foo << ":";
        //printf("%02X", buf[j]);
        foo << std::hex << buf[j];
    }
//    printf("\n");
    foo << std::endl;

    debug_msg(foo.str().c_str());

    char buf_decoded[255];
    cobs_decode_result result = cobs_decode(&buf_decoded, 255, &buf, i); //this might need to be i-1, along with the for loop below

    if(result.status != COBS_DECODE_OK){
        error_msg("COBS decode failed! Status code: %d\n", result.status);
        return;
    }


    foo.clear();
    foo << "Received " << std::hex << (i) << " COBS-decoded bytes: ";
    //printf("Received %d COBS-decoded bytes: ", result.out_len);
    for(int j = 0; j < result.out_len; j++){
        if (j > 0)
            foo << ":";
        //printf("%02X", buf[j]);
        foo << std::hex << buf[j];
    }
//    printf("\n");
    foo << std::endl;

    debug_msg(foo.str().c_str());

    pb_istream_s istream = pb_istream_from_buffer((uint8_t*) &buf_decoded, result.out_len);

    Status status_tmp = Status_init_zero;
    bool success = pb_decode(&istream, Status_fields, (void*) &status_tmp);
    if(!success){
        error_msg("Protobuf decode failed!\n");
        return;
    }
    if(status_tmp.counter == -1){
        status_tmp.counter = status.lock()->counter;
    }
    *status.lock() = status_tmp;
}

void foo(){
    printf("got inside foo!\n");
}

void Arduino::write_command(){
    char buff[255];
    Command command_tmp = *command.lock();

    pb_ostream_s ostream = pb_ostream_from_buffer((uint8_t*) &buff, 255);

    bool success = pb_encode(&ostream, Command_fields, &command_tmp);


    std::ostringstream foo;
    foo << "Generated " << ostream.bytes_written << " protobuf-encoded bytes: ";
    //printf("generated %d protobuf-encoded bytes: ", ostream.bytes_written);
    for(int j = 0; j < ostream.bytes_written; j++){
        if (j > 0)
            foo << ":";
        //printf("%02X", buff[j]);
        foo << std::hex << buff[j];
    }
    //printf("\n");
    foo << std::endl;
    debug_msg(foo.str().c_str());


    if(!success){
        error_msg("Protobuf encode failed!\n");
    }

    char buff_cobs[255];

    debug_msg("ostream bytes written: %d\n", ostream.bytes_written);

    cobs_encode_result result = cobs_encode_mine(&buff_cobs, 255, &buff, ostream.bytes_written);

    if(result.status != COBS_ENCODE_OK){
        error_msg("COBS encode failed! Status code: %d\n", result.status);
    }


    printf("generated %d COBS-encoded bytes: ", result.out_len);
    for(int j = 0; j < result.out_len; j++){
        if (j > 0) printf(":");
        printf("%02X", buff_cobs[j]);
    }
    printf("\n");
    //null terminate the message
    buff_cobs[result.out_len] = '\0';


    serial_port.write((uint8_t*)&buff_cobs, result.out_len+1);
}

void Arduino::comms_thread(){
    pros::delay(10);
    this->serial_port.flush();
    while(true){
        this->serial_port.flush();
        log_msg("Timestamp: %ld\n", pros::millis());
        log_msg("Attempting to receive status...\n");
        get_status();
        //printf("Counter: %ld\n", getCounter());
        pros::delay(2);
        log_msg("Attempting to send command...\n");
//        printf("D1: %d\n", command.lock()->out_1);
//        printf("D2: %d\n", command.lock()->out_2);
        write_command();
        pros::delay(1); //TODO: remove this
    }
}

bool Arduino::getD1(){
    return status.lock()->in_1;
}

bool Arduino::getD2(){
    return status.lock()->in_2;
}

void Arduino::setD1(bool value){
    command.lock()->out_1 = value;
}

void Arduino::setD2(bool value){
    command.lock()->out_2 = value;
}

int32_t Arduino::getCounter(){
    return status.lock()->counter;
}

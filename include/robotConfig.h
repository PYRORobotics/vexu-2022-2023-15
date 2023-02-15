
#include "api.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include <type_traits>
#include "AMT21.h"
#include "navX.h"

    inline pros::Motor left_front0_mtr(6);
	inline pros::Motor left_front1_mtr(7);
	inline pros::Motor left_back0_mtr(10);
	inline pros::Motor left_back1_mtr(9);
//    inline pros::Motor right_front0_mtr(5, true);
//    inline pros::Motor right_front1_mtr(4, true);
//    inline pros::Motor right_back0_mtr(3, true);
//    inline pros::Motor right_back1_mtr(2, true);

    inline pros::Motor right_front0_mtr(5);
    inline pros::Motor right_front1_mtr(1);
    inline pros::Motor right_back0_mtr(3);
    inline pros::Motor right_back1_mtr(2);

    //inline pros::Motor flywheelBig(14);
    inline sylib::Motor flywheelBig(14);
    //inline pros::Motor flywheelSmall(15);
    inline sylib::Motor flywheelSmall(15);

    inline pros::Motor intake(17, pros::E_MOTOR_GEAR_GREEN);
	/*inline pros::Rotation right_encoder(21);
	inline pros::Rotation left_encoder(20);
	inline pros::Rotation center_encoder(19);*/
    inline pros::Imu imu2(16);
    inline navX imu1(20);
	//inline pros::ADIEncoder up('a', 'b', true);
	//inline pros::ADIEncoder sideways('c', 'd', false);
	inline AMT21 up(13, 0x5c,false);
    inline AMT21 up2(19, 0x58, true);
	inline AMT21 sideways(11, 0x54, true);
    inline pros::Controller master(pros::E_CONTROLLER_MASTER);

    inline pros::ADIDigitalOut indexer ('E');
    inline pros::ADIDigitalOut endgame ('H');


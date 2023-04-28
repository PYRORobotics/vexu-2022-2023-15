
#include "api.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include <type_traits>
#include "AMT21.h"
#include "navX.h"

    inline pros::Motor left_front0_mtr(10);//PyroSticker Bot is port 12 and the non sticker one is port 10
	inline pros::Motor left_front1_mtr(9);
	inline pros::Motor left_back0_mtr(7);
	inline pros::Motor left_back1_mtr(6);

    inline pros::Motor right_front0_mtr(1);
    inline pros::Motor right_front1_mtr(2);
    inline pros::Motor right_back0_mtr(3);
    inline pros::Motor right_back1_mtr(5);

    inline sylib::Motor flywheelBig(14);
    inline sylib::Motor flywheelSmall(15);

    inline pros::Motor intake(13, pros::E_MOTOR_GEAR_GREEN);
    inline pros::Motor roller(11);
	/*inline pros::Rotation right_encoder(21);
	inline pros::Rotation left_encoder(20);
	inline pros::Rotation center_encoder(19);*/
    inline pros::Imu imu2(17);//White
    inline navX imu1(16);
	//inline pros::ADIEncoder up('a', 'b', true);
	//inline pros::ADIEncoder sideways('c', 'd', false);
	inline AMT21 up(20, 0x54,false);//Blue IF UP IS TRUE AND UP2 IS FALSE STRAFE VALUE IN ROBOTCONTROL LINE 42(relStrafe) SHOULD BE SIGN SWITCHED
    inline AMT21 up2(19, 0x54, true);//Red - Pyrosticker bot is 18, the other one is 19
	inline AMT21 sideways(18, 0x54, true); //Pink - pyro sticker one is 17, the other one is 18
    inline pros::Controller master(pros::E_CONTROLLER_MASTER);

    inline pros::ADIDigitalOut blooper ('G');
    inline pros::ADIDigitalOut tilter ('F');
    inline pros::ADIDigitalOut endgame ('A');
    inline pros::ADIDigitalOut zapper ('H');


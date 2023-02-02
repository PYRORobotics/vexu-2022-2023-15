#ifndef path_h
#define path_h


//Refrence angle is +Y
#include "main.h"
#include "cartesian.h"
#include "polar.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "robotPose.h"
class Cartesian;
class RobotPose;
class Path{
    private: 
    static Cartesian interpolateBtw(Cartesian p1, Cartesian p2, double progress);
    static robotPose interpolateBtw(robotPose p1, robotPose p2, double progress);
    static robotPose qbezier(robotPose p1, robotPose p2, robotPose p3, double progress);
    public:
    Path(); 
    Path(std::vector<robotPose>);
    static std::vector<robotPose> path1;
    static std::vector<robotPose> generateStraightPath(robotPose point1, robotPose point2, int points);
    //linearly scales the heading from point 1 to point 2
    static std::vector<robotPose> qbezierManualHeading(robotPose start, robotPose controlPoint, robotPose endPoint, int points);
    //robot points towards direction of travel, with an option of applying an offset
    static  std::vector<robotPose> qbezierAutomaticHeading(robotPose start, robotPose controlPoint, robotPose endPoint, int points, okapi::QAngle offset);
    
};

#endif

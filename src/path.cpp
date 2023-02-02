#include "path.h"
#include "polar.h"
#include "robotPose.h"
#include "cartesian.h"
using std::vector;

Path::Path() {}
Path::Path(std::vector<robotPose> input) {
    this->path1 = input;
}
//returns a cartesian between the two input points, 0 is at p1, 1 is at p2
Cartesian Path::interpolateBtw(Cartesian p1, Cartesian p2, double progress) {
    Cartesian point1 = Cartesian(p1);
    Cartesian point2 = Cartesian(p2);
    point1.scale(1-progress);
    point2.scale(progress);
    point1.add(point2);
    return point1;
}
//returns a robotPose between the two input robotPoses, 0 is at p1, 1 is at p2
//heading of the output linearly changes from p1 to p2
robotPose Path::interpolateBtw(robotPose p1, robotPose p2, double progress) {
    Cartesian point1 = Cartesian(p1.position);
    Cartesian point2 = Cartesian(p2.position);
    point1.scale(1-progress);
    point2.scale(progress);
    point1.add(point2);
    robotPose output = robotPose(point1, p1.heading*(1-progress)+p2.heading*(progress));
    return output;
}
//linearly changes robot heading from p1 to p3
robotPose Path::qbezier(robotPose p1, robotPose p2, robotPose p3, double progress) {
    Cartesian output = interpolateBtw( interpolateBtw(p1.position,p2.position,progress),interpolateBtw(p2.position,p3.position,progress),progress);
        return robotPose(output, p1.heading*(1-progress)+p3.heading*(progress));
    
}

std::vector<robotPose> Path::generateStraightPath(robotPose point1, robotPose point2, int points) {
    vector<robotPose> output;
    int counter = 0;
    double stepsize = 1.0/(points-1);
    double progress = 0;
    while(points>counter) {
        output.push_back(interpolateBtw(point1,point2,progress));
        counter++;
        progress+=stepsize;
    }
    return output;
}
//This creates a bezier curve and smoothly transitions the heading at start to the heading given at endPoint.
std::vector<robotPose> Path::qbezierManualHeading(robotPose start, robotPose controlPoint, robotPose endPoint, int points) {
    vector<robotPose> output;
    int counter = 0;
    double stepsize = 1.0/(points-1);
    double progress = 0;
    while(points > counter) {
        output.push_back(qbezier(start, controlPoint, endPoint, progress));
        counter++;
        progress += stepsize;
    }
    return output;
}
//This creates a bezier curve wile making the robot look in the direction it is traveling with the option of adding an offset.
std::vector<robotPose> Path::qbezierAutomaticHeading(robotPose start, robotPose controlPoint, robotPose endPoint, int points, okapi::QAngle offset){
    vector<robotPose> output;
    okapi::QAngle heading;
    int counter = 0;
    double stepsize = 1.0/(points-1);
    double progress = 0;
    Cartesian p_now = Cartesian(qbezier(start, controlPoint, endPoint, progress).position);
    Cartesian p_to = Cartesian(qbezier(start, controlPoint, endPoint, progress).position);
    p_now.scale(-1);
    p_to.add(p_now);
    while(points > counter) {
        //check where the bot is looking within one quadrent
        if(p_to.x == 0_in && p_to.y == 0_in){
            heading = offset;
        }
        else if(p_to.x == 0_in){
            heading = 90_deg + offset;
        }
        else {
          heading = ((atan(p_to.y.convert(okapi::inch) / p_to.x.convert(okapi::inch)) * 1_rad) + offset);  
        }
        //check which half the bot is heading
        if(p_to.y < 0_in) {
            heading += 180_deg;
        }
        output.push_back(robotPose(qbezier(start, controlPoint, endPoint, progress).position, heading));
        counter += 1;
        progress += stepsize;
    }
    return output;
}
//IDK HOW TO PROPERLY USE OKAPI
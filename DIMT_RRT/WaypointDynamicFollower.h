//
// Created by Olga Mikheeva on 01/02/16.
//

#include <vector>
#include "dp_structs.h"

#ifndef WAYPOINTDYNAMICFOLLOWER_H
#define WAYPOINTDYNAMICFOLLOWER_H

class WaypointDynamicFollower {
private:
    //segments of different acceleration
    double tick;
    double time = 0;
    unsigned long nsx; //number of segments X
    unsigned long nsy; //number of segments Y
    std::vector<segment> accX;
    std::vector<segment> accY;
    std::vector<double> ssVelX; //velocities at start of each segment of X
    std::vector<double> ssVelY; //velocities at start of each segment of Y
    std::vector<double> segX; //coordinate at the start of each segment of X
    std::vector<double> segY; //coordinate at the start of each segment of Y
    std::vector<double> timeX; //time of the start of each segment of X
    std::vector<double> timeY; //time of the start of each segment of Y
    int csx = 0; //current segment of X
    int csy = 0; //current segment of Y
    std::pair<double, double> start_point;
    std::pair<double, double> start_velocity;
    void ComputeSSVelocities();
    void ComputeStartSegmentCoord();
    void ComputeAccumulatedTime();

public:
    WaypointDynamicFollower() {};
    WaypointDynamicFollower(std::pair<double,double> start, std::pair<double,double> start_vel,
                            std::pair<std::vector<segment>,std::vector<segment>> segmentTrajectory,
                            double tick);
    std::pair<double, double> getNextCoordinates();
};

#endif
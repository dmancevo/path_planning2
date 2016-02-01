//
// Created by Olga Mikheeva on 01/02/16.
//

#include <math.h>
#include <iostream>
#include "WaypointDynamicFollower.h"



WaypointDynamicFollower::WaypointDynamicFollower(std::pair<double,double> start_coord,
                                                 std::pair<double,double> start_vel,
                                                 std::pair<std::vector<segment>, std::vector<segment>> segmentTrajectory,
                                                 double tick) {

    this->tick = tick;
    this->start_point = start_coord;
    this->start_velocity = start_vel;
    this->accX = segmentTrajectory.first;
    this->accY = segmentTrajectory.second;
    this->nsx = this->accX.size();
    this->nsy = this->accY.size();
    this->ssVelX = std::vector<double>(this->nsx +1);
    this->ssVelY = std::vector<double>(this->nsy + 1);
    this->segX = std::vector<double>(this->nsx + 1);
    this->segY = std::vector<double>(this->nsy + 1);
    this->timeX = std::vector<double>(this->nsx + 1);
    this->timeY = std::vector<double>(this->nsy + 1);
    this->ComputeSSVelocities();
    this->ComputeStartSegmentCoord();
    this->ComputeAccumulatedTime();
}

void WaypointDynamicFollower::ComputeSSVelocities() {
    this->ssVelX.at(0) = this->start_velocity.first;
    this->ssVelY.at(0) = this->start_velocity.second;
    for (unsigned int j = 1; j < this->nsx + 1; ++j) {
        this->ssVelX.at(j) =  this->ssVelX.at(j-1) + this->accX.at(j-1).acc * this->accX.at(j-1).time;
    }
    for (unsigned int j = 1; j < this->nsy + 1; ++j) {
        this->ssVelY.at(j) =  this->ssVelY.at(j-1) + this->accY.at(j-1).acc * this->accY.at(j-1).time;
    }
}

void WaypointDynamicFollower::ComputeStartSegmentCoord() {
    this->segX.at(0) = this->start_point.first;
    this->segY.at(0) = this->start_point.second;
    for (unsigned int j = 1; j < this->nsx + 1; ++j) {
        this->segX.at(j) =  0.5 * this->accX.at(j-1).acc * pow(this->accX.at(j-1).time, 2)
                            + this->ssVelX.at(j-1) * this->accX.at(j-1).time + this->segX.at(j-1);
    }
    for (unsigned int j = 1; j < this->nsy + 1; ++j) {
        this->segY.at(j) =  0.5 * this->accY.at(j-1).acc * pow(this->accY.at(j-1).time, 2)
                            + this->ssVelY.at(j-1) * this->accY.at(j-1).time + this->segY.at(j-1);
    }
}

void WaypointDynamicFollower::ComputeAccumulatedTime() {
    this->timeX.at(0) = 0;
    this->timeY.at(0) = 0;
    for (unsigned int j = 1; j < this->nsx + 1; ++j) {
        this->timeX.at(j) = this->timeX.at(j-1) + this->accX.at(j-1).time;
    }
    for (unsigned int j = 1; j < this->nsy + 1; ++j) {
        this->timeY.at(j) = this->timeY.at(j-1) + this->accY.at(j-1).time;
    }
}

std::pair<double, double> WaypointDynamicFollower::getNextCoordinates() {
    this->time += this->tick;
    std::pair<double, double> coordinates;
    double timeInSegX;
    double timeInSegY;
    double a_x, a_y;

    //detect current segment
    if (this->csx < this->nsx && this->time > this->timeX[this->csx + 1]) {
        this->csx += 1;
    }
    if (this->csy < this->nsy && this->time > this->timeY[this->csy + 1]) {
        this->csy += 1;
    }

    timeInSegX = this->time - this->timeX[this->csx];
    timeInSegY = this->time - this->timeY[this->csy];

    if (this->csx == this->nsx) {
        a_x = 0;
    } else {
        a_x = this->accX[this->csx].acc;
    }
    if (this->csy == this->nsy) {
        a_y = 0;
    } else {
        a_y = this->accY[this->csy].acc;
    }
    coordinates.first = 0.5 * a_x * pow(timeInSegX, 2) + this->ssVelX[this->csx] * timeInSegX + this->segX[this->csx];

    coordinates.second = 0.5 * a_y * pow(timeInSegY, 2) + this->ssVelY[this->csy] * timeInSegY + this->segY[this->csy];

    return coordinates;
}
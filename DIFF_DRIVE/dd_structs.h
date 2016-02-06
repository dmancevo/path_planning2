//
// Created by Olga Mikheeva on 05/02/16.
//

#ifndef DIFF_DRIVE_DD_STRUCTS_H
#define DIFF_DRIVE_DD_STRUCTS_H

#include <vector>

struct diffDrivePose {
    double x;
    double y;
    double theta;
};

struct diffDriveWP {
    double theta_init;
    double vel_max;
    double w_max; //angular velocity
    std::vector<std::pair<double,double>> points;
};

struct ddParams {
    double v;
    double w;
    double phi;
    double angle_diff;
};
#endif //DIFF_DRIVE_DD_STRUCTS_H

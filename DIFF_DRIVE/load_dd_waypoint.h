//
// Created by Olga Mikheeva on 05/02/16.
//
#include <string>
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include "dd_structs.h"

#ifndef DIFF_DRIVE_LOAD_DD_WAYPOINT_H
#define DIFF_DRIVE_LOAD_DD_WAYPOINT_H

/*
 * File structure:
 * v_max
 * w_max
 * theta_init
 * x0 y0
 * x1 y1
 * ...
 */
diffDriveWP loadDiffDriveWaipoint(std::string path_to_txt) {
    std::ifstream file(path_to_txt);
    diffDriveWP result;
    std::string str;
    std::getline(file, str);
    result.vel_max = std::stod(str);
    std::getline(file, str);
    result.w_max = std::stod(str);
    std::getline(file, str);
    result.theta_init = std::stod(str);

    while (std::getline(file, str))
    {
        std::pair<double,double> point;
        std::vector<std::string> numbers;
        boost::split(numbers, str, boost::is_any_of(", "), boost::token_compress_on);

        point.first = std::stod(numbers[0]);
        point.second = std::stod(numbers[1]);

        result.points.push_back(point);
    }
    return result;
}

#endif //DIFF_DRIVE_LOAD_DD_WAYPOINT_H

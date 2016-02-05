//
// Created by Olga Mikheeva on 01/02/16.
//

#ifndef DIMT_RRT_DPOINTPATHPLANNER_H
#define DIMT_RRT_DPOINTPATHPLANNER_H


#include <utility>
#include <vector>
#include <random>
#include <fstream>
#include "node.h"
#include "dp_structs.h"

class DPointPathPlanner {
public:
    DPointPathPlanner(double x_min, double x_max,
                      double y_min, double y_max,
                      double acc_max, double vel_max);
    std::pair<std::vector<segment>,std::vector<segment>> findPath(std::pair<double, double> start_coord,
                                                                  std::pair<double, double> finish_coord,
                                                                  std::pair<double, double> start_vel,
                                                                  std::pair<double, double> finish_vel);
private:
    double x_min, x_max; //lower and upper bounds of x
    double y_min, y_max; //lower and upper bounds of y
    double acc_max; //acceleration limit
    double vel_max; //velocity limit
    std::pair<double, double> start_coord;
    std::pair<double, double> finish_coord;
    std::pair<double, double> start_vel;
    std::pair<double, double> finish_vel;
    std::vector<node*> nodes;
    double endgame_region = 0.25;
    double neighbourhood = 0.5;
    double delta_time_min = 5;
    double delta_time_max = 20;
    double delta_tr_check = 0.1;
    unsigned long node_limit = 10000;
    std::vector<double> weights;
    int getRandomNodeID();
    std::pair<double, double> getRandomAcc();
    double getRandomTime();
    node* propagate(node* parent, std::pair<double, double> acc, double time);
    bool isValidTrajectory(node *parent, node *new_node);
    void updateWeights(node *new_node);
    std::fstream fs;
    std::fstream fs2;
    std::string temp;
};


#endif //DIMT_RRT_DPOINTPATHPLANNER_H

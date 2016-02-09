//
// Created by Olga Mikheeva on 05/02/16.
//

#ifndef DIFF_DRIVE_DDWPFOLLOWER_H
#define DIFF_DRIVE_DDWPFOLLOWER_H

#import "dd_structs.h"
#include "../V_graph/V_graph.h"

const double EPS_OBST = 0.001;
const double EPS_ANG = 0.01;
const double EPS_COORD = 0.001;

class DDWPFollower {
private:
    V_graph *vis_graph;
    double tick;
    double theta; // current angle
    double x, y; //current coordinates
    double v_max;
    double w_max;
    double time = 0;
    std::vector<std::pair<double, double>> points;
    unsigned long size;
    bool limitedField;
    bool atNewPoint = true;
    double  ang_diff; //current difference between theta and phi
    double coord_diff; //current difference between current position and next point in the list
    double v, w; //current vel and angular vel
    std::pair<double, double> x_lim;
    std::pair<double, double> y_lim;
    double simplifyAngle(double rad);
    double getPhi(std::pair<double, double> p_curr, std::pair<double, double> p_next);
    bool inLimit(double x, double y);
    bool inObstacle(std::pair<double,double> prev_p, std::pair<double,double> new_p);
    ddParams computeAngles(double theta, std::pair<double, double> p_curr,
                           std::pair<double, double> p_next, bool turning);
    ddParams computePramMaxVelInLimits(double theta, std::pair<double, double> p_curr,
                           std::pair<double, double> p_next);
public:
    DDWPFollower(double tick, std::vector<std::pair<double, double>> points, double theta,
                 double v_max, double w_max, V_graph *vis_graph, bool limitedField = false,
                 std::pair<double, double> x_lim = {0,0}, std::pair<double, double> y_lim = {0,0});
    DDWPFollower(V_graph *vis_graph){ this->vis_graph = vis_graph;};
    diffDrivePose getNextPose();
    double getTime() {return this->time;};
    int next_point_id = -1;
    bool finished = false;
};


#endif //DIFF_DRIVE_DDWPFOLLOWER_H

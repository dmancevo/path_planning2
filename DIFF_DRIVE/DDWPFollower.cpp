//
// Created by Olga Mikheeva on 05/02/16.
//

#include "DDWPFollower.h"
#include <math.h>
#include <iostream>


DDWPFollower::DDWPFollower(double tick, std::vector<std::pair<double, double>> points, double theta,
                           double v_max, double w_max, bool limitedField,
                           std::pair<double, double> x_lim, std::pair<double, double> y_lim) {
    this->tick = tick;
    this->points = points;
    this->v_max = v_max;
    this->w_max = w_max;
    this->limitedField = limitedField;
    this->x_lim = x_lim;
    this->y_lim = y_lim;
    this->size = this->points.size();

    //theta
    if (theta < 0) {
        theta = - fmod(fabs(theta), 2.0 * M_PI) + 2.0 * M_PI;
    }
    this->theta = this->simplifyAngle(theta);

    if (this->size > 0) {
        this->x = points[0].first;
        this->y = points[0].second;
    }

    if (this->size > 1) {
        this->next_point_id = 1;
        ddParams params = this->computePramMaxVelInLimits(this->theta,
                                                          {this->x, this->y},
                                                          this->points[1]);
        this->ang_diff = params.angle_diff;
        this->v = params.v;
        this->w = params.w;
        this->coord_diff  = sqrt(pow(this->points[1].first - this->x, 2) + pow(this->points[1].second - this->y, 2));
    }


}

diffDrivePose DDWPFollower::getNextPose() {
    diffDrivePose pose = diffDrivePose();

    if (this->next_point_id < 0) {
        pose.theta = this->theta;
        pose.x = this->x;
        pose.y = this->y;
        if (!this->finished) {
            std::cout << "finished, time = " << this->getTime() << "\n";
            this->finished = true;
        }
        return pose;
    }

    this->x += this->v * cos(this->theta) * this->tick;
    this->y += this->v * sin(this->theta) * this->tick;
    this->theta += this->w * this->tick;
    this->theta = this->simplifyAngle(this->theta);

    this->coord_diff = sqrt(pow(this->points[this->next_point_id].first - this->x, 2)
                            + pow(this->points[this->next_point_id].second - this->y, 2));

    //if at the next point
    if (this->coord_diff < EPS_COORD) {
        if (this->next_point_id == this->size - 1) {
            this->next_point_id = -1; //arrived at final point
        } else {
            this->next_point_id += 1;
            this->coord_diff = sqrt(pow(this->points[this->next_point_id].first - this->x, 2)
                                    + pow(this->points[this->next_point_id].second - this->y, 2));
            ddParams params = this->computePramMaxVelInLimits(this->theta,
                                                              {this->x, this->y},
                                                              this->points[this->next_point_id]);
            this->ang_diff = params.angle_diff;
            this->v = params.v;
            this->w = params.w;
        }
    } else {
        ddParams params = this->computePramMaxVelInLimits(this->theta, {this->x, this->y},
                                                          this->points[this->next_point_id]);
        this->ang_diff = params.angle_diff;
        this->v = params.v;
        this->w = params.w;
    }

    this->time += this->tick; //increase time

    pose.theta = this->theta;
    pose.x = this->x;
    pose.y = this->y;
    return pose;
}


double DDWPFollower::simplifyAngle(double rad) {
    if (fabs(rad) > M_PI) {
        if (rad > 0) {
            rad = -M_PI + fmod(fabs(rad), M_PI);
        } else {
            rad = M_PI - fmod(fabs(rad), M_PI);
        }
    }
    return rad;
}

double DDWPFollower::getPhi(std::pair<double, double> p_curr, std::pair<double, double> p_next) {
    double dx = p_next.first - p_curr.first;
    double dy = p_next.second - p_curr.second;
    double phi = 0;
    if (dx == 0.0) {
        if (dy > 0) {
            phi = M_PI/2.0;
        } else if (dy < 0) {
            phi = 3.0/2.0 * M_PI;
        } else {
            phi = 0;
        }
    } else if (dy == 0.0) {
        if (dx > 0) {
            phi = 0;
        } else {
            phi = M_PI;
        }
    } else {
        if (dx > 0 && dy > 0) {
            phi = atan(dy/dx);
        } else if (dx < 0 && dy > 0) {
            phi = M_PI - atan(fabs(dy/dx));
        } else if (dx < 0 && dy < 0) {
            phi = M_PI + atan(fabs(dy/dx));
        } else {
            phi = 2.0 * M_PI - atan(fabs(dy/dx));
        }
    }
    return this->simplifyAngle(phi);
}

bool DDWPFollower::inLimit(double x, double y) {
    if (this->limitedField) {
        bool xInLimit = (x >= this->x_lim.first - EPS_OBST) &&  (x <= this->x_lim.second + EPS_OBST);
        bool yInLimit = (y >= this->y_lim.first - EPS_OBST) &&  (y <= this->y_lim.second + EPS_OBST);
        return xInLimit && yInLimit;
    } else {
        return true;
    }
}

ddParams DDWPFollower::computeAngles(double theta, std::pair<double, double> p_curr, std::pair<double, double> p_next,
                                     bool turning) {
    double phi = this->getPhi(p_curr, p_next);
    double d1, d2, min_angle, angle_diff;
    double v, w;
    double a1, a2;
    bool forward;

    d1 = fabs(phi - theta);
    if (phi > theta) {
        d2 = fabs(phi - 2.0 * M_PI - theta);
    } else {
        d2 = fabs(phi - (theta - 2.0 * M_PI));
    }
    min_angle = d1;
    if (d2 < d1) {
        min_angle = d2;
        if (phi > theta) {
            phi = phi - 2.0 * M_PI;
        } else {
            phi = phi + 2.0 * M_PI;
        }
    }

    forward = (min_angle <= M_PI/2.0);

    if (forward) {
        w = (phi >= theta)? this->w_max : - this->w_max;
    } else {
        a1 = phi - M_PI;
        a2 = phi + M_PI;
        phi = (fabs(a1 - theta) < fabs(a2 - theta))? a1 : a2;
        w = (phi >= theta)? this->w_max : -this->w_max;
    }

    v = (forward)? this->v_max : -this->v_max;

    angle_diff = phi - theta;


    if (turning) {
        //take into account radius and decrease v if necessary
        double v_rad, v_res;
        double denom = (p_next.second - p_curr.second) * cos(theta) - (p_next.first - p_curr.first) * sin(theta);
        if (denom == 0.0) {
            v_rad = v;
        } else {
            v_rad = w / 2.0 * (pow(p_next.first - p_curr.first, 2) + pow(p_next.second - p_curr.second, 2)) / denom;
        }
        if (v > 0) {
            v_res = (v < v_rad)? v : v_rad; //min
        } else {
            v_res = (v > v_rad)? v : v_rad; //max
        }
        v = v_res;
    }

    if (fabs(angle_diff) < EPS_ANG) {
        w = 0;
    }

    ddParams params = ddParams();
    params.angle_diff = angle_diff;
    params.phi = phi;
    params.v = v;
    params.w = w;
    return params;
}

ddParams DDWPFollower::computePramMaxVelInLimits(double theta, std::pair<double, double> p_curr,
                                                 std::pair<double, double> p_next) {
    ddParams params = this->computeAngles(theta, p_curr, p_next, true);
    int n = 10;
    double v_step = params.v/double(10);
    double v_c = params.v;
    double angle_diff;
    double x, y;
    double theta1;
    bool inLimits;
    ddParams param_temp;

    for (int i = 0; i < n; ++i) {
        angle_diff = params.angle_diff;
        x = p_curr.first;
        y = p_curr.second;
        theta1 = theta;
        inLimits = true;

        while (fabs(angle_diff) > EPS_ANG) {
            x += v_c * cos(theta1) * this->tick;
            y += v_c * sin(theta1) * this->tick;
            inLimits = this->inLimit(x, y);
            if (!inLimits) {
                std::cout<<"not in lim\n";
                break;
            }
            theta1 += this->tick * params.w;
            theta1 = this->simplifyAngle(theta1);

            param_temp = this->computeAngles(theta1, {x, y}, p_next, true);

            if (fabs(param_temp.angle_diff) > fabs(angle_diff)) {
                break;
            } else {
                angle_diff = param_temp.angle_diff;
            }
        }

        if (inLimits) {
            break;
        } else {
            v_c -= v_step;
        }
    }

    params.v = v_c;

    return params;
}

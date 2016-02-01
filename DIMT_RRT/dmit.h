//
// Created by Olga Mikheeva on 28/01/16.
//

#include <vector>
#include <tuple>
#include <random>
#include <iostream>
#include "dp_structs.h"


std::pair<double, double> maxForEach(std::pair<double, double> p1, std::pair<double, double> p2, double max) {
    max = fabs(max);
    double x_delta = fabs(p2.first - p1.first);
    double y_delta = fabs(p2.second - p1.second);
    std::pair<double, double> maxima;

    if (x_delta != 0 && y_delta != 0) {
        double b = x_delta/y_delta;
        maxima.second = max/sqrt(b*b + 1);
        maxima.first = b * maxima.second;
    } else if (x_delta == 0 && y_delta == 0) {
        double max1 = sqrt(max/2);
        maxima.first = max1;
        maxima.second = max1;
    } else if (x_delta != 0) {
        maxima.first = max;
        maxima.second = 0;
    } else {
        maxima.first = 0;
        maxima.second = max;
    }

    return maxima;
};


std::pair<double,std::pair<double,double>> findMinTimeOneDOF(double p1, double p2,
                                                             double v1, double v2,
                                                             double v_max, double a_max) {
    //determine the sign (9) - (12)
    double delta_p_acc_x = 0.5 * (v1 + v2) * fabs(v2 - v1) / a_max;
    int sign = (p2 - p1 - delta_p_acc_x < 0)? -1 : 1;
    double a1 = sign * a_max;
    double a2 = -a1;
    double v_lim = sign * v_max;
    double T;
    std::pair<double, double> T_inf(0, 0);

    //(13)
    double a = a1;
    double b = 2 * v1;
    double c = (pow(v2, 2) - pow(v1, 2))/(2 * a2) - (p2 - p1);
    double q, t1, t2, v_t_a1;
    double t_a1, t_v, t_a2; //duration of time segments

    int sign_a = (a < 0)? -1 : 1;
    int sign_b = (b < 0)? -1 : 1;


    q = -0.5 * (b + sign_b * sqrt(pow(b, 2) - 4 * a * c));

    t1 = q / a;
    t2 = c / q;

    t_a1 = (sign_a == sign_b) ? t2 : t1;
    t_a2 = (v2 - v1) / a2 + t_a1;
    t_v = 0;

    //check for velocity limits at t_a1
    v_t_a1 = v1 + a1 * t_a1;

    if (fabs(v_t_a1) > v_max) {
        //(15)-(17)
        t_a1 = (v_lim - v1) / a1;
        t_a2 = (v2 - v_lim) / a2;
        t_v = (pow(v1, 2) + pow(v2, 2) - 2 * pow(v_lim, 2)) / (2 * v_lim * a1) + (p2 - p1) / v_lim;
    }

    T = t_a1 + t_a2 + t_v;



    //compute limits of infeasible time interval
    a1 = -sign * a_max;
    a2 = -a1;
    v_lim = -sign * v_max;
    a = a1;
    b = 2 * v1;
    c = (pow(v2, 2) - pow(v1, 2))/(2 * a2) - (p2 - p1);

    sign_a = (a < 0)? -1 : 1;
    sign_b = (b < 0)? -1 : 1;

    q = -0.5 * (b + sign_b * sqrt(pow(b, 2) - 4 * a * c));

    if (!isnan(q)) {

        t1 = q / a;
        t2 = c / q;

        t_a1 = (sign_a == sign_b) ? t1 : t2; //min - lower bound of infeasible interval
        t_a2 = (v2 - v1) / a2 + t_a1;
        t_v = 0;
        T_inf.first = t_a1 + t_a2 + t_v;

        t_a1 = (sign_a == sign_b) ? t2 : t1; //max - upper bound of infeasible interval
        t_a2 = (v2 - v1) / a2 + t_a1;
        t_v = 0;

        //check for velocity limits at t_a1 for upper bound
        v_t_a1 = v1 + a1 * t_a1;

        if (fabs(v_t_a1) > v_max) {
            //(15)-(17)
            t_a1 = (v_lim - v1) / a1;
            t_a2 = (v2 - v_lim) / a2;
            t_v = (pow(v1, 2) + pow(v2, 2) - 2 * pow(v_lim, 2)) / (2 * v_lim * a1) + (p2 - p1) / v_lim;
        }

        T_inf.second = t_a1 + t_a2 + t_v;
    }

    return std::pair<double, std::pair<double,double>> (T, T_inf);
}

double findMinOverallTime(std::vector<std::pair<double, std::pair<double,double>>> DOFs) {
    double T_min = 0; //max of min times
    for (auto dof: DOFs) {
        T_min = (T_min < dof.first)? dof.first : T_min;
    }

    //check for infeasible intervals

    bool changed = 1;

    while (changed) {
        changed = 0;
        for (auto dof: DOFs) {
            if (T_min > dof.second.first && T_min < dof.second.second) {
                T_min = dof.second.second;
                changed = 1;
            }
        }
    }

    return T_min;
}

std::vector<segment> findFixedTimeTrajectory(
        double p1, double p2, double v1, double v2, double v_max, double a_max, double T) {
    double a = pow(T, 2);
    double b = 2 * T * (v1 + v2) - 4 * (p2 - p1);
    double c = - pow(v2 - v1, 2);

    int sign_a = (a < 0)? -1 : 1;
    int sign_b = (b < 0)? -1 : 1;

    double q = -0.5 * (b + sign_b * sqrt(pow(b, 2) - 4 * a * c));
    double a1_1 = q / a;
    double a1_2 = c / q;

    double a1 = (fabs(a1_1) > fabs(a1_2))? a1_1 : a1_2;
    double a2 = -a1;
    double t_a1 = 0.5 * ((v2 - v1)/a1 + T);
    double t_a2 = T - t_a1;
    double t_v = 0;
    double v_lim = 0;

    //check for velocity limits at t_a1 for upper bound
    double v_t_a1 = v1 + a1 * t_a1;


    if (fabs(v_t_a1) > v_max) {
        int sign_a1 = (a1 < 0)? -1 : 1;
        v_lim = sign_a1 * v_max;
        a1 = (pow(v_lim - v1, 2) + pow(v_lim - v2, 2))/(2 * (v_lim * T - (p2 - p1)));
        a2 = -a1;
        //(15)-(17)
        t_a1 = (v_lim - v1) / a1;
        t_a2 = (v2 - v_lim) / a2;
        if (t_a1 < 0) {
        }
        if (t_a2 < 0) {
        }
        t_v = (pow(v1, 2) + pow(v2, 2) - 2 * pow(v_lim, 2)) / (2 * v_lim * a1) + (p2 - p1) / v_lim;
    }

    std::vector<segment> result;
    result.push_back(segment(t_a1, a1));
    result.push_back(segment(t_v, 0));
    result.push_back(segment(t_a2, a2));
    return result;
}

trajectory findMinTime(std::pair<double, double> p1, std::pair<double, double> p2,
                   std::pair<double, double> v1, std::pair<double, double> v2,
                   double v_maximum, double a_maximum) {

    std::pair<double, double> v_max(v_maximum/sqrt(2), v_maximum/sqrt(2));
    std::pair<double, double> a_max = maxForEach(v1, v2, a_maximum);
    std::pair<double,std::pair<double,double>> Tx = findMinTimeOneDOF(p1.first, p2.first, v1.first, v2.first,
                                                                      v_max.first, a_max.first);
    std::pair<double,std::pair<double,double>> Ty = findMinTimeOneDOF(p1.second, p2.second, v1.second, v2.second,
                                                                      v_max.second, a_max.second);

    //compute overall min time
    std::vector<std::pair<double, std::pair<double,double>>> DOFs;
    DOFs.push_back(Tx);
    DOFs.push_back(Ty);
    double T_min = findMinOverallTime(DOFs);

    std::vector<segment> Trx = findFixedTimeTrajectory(p1.first, p2.first, v1.first, v2.first,
                                                                         v_max.first, a_max.first, T_min);
    std::vector<segment> Try = findFixedTimeTrajectory(p1.second, p2.second, v1.second, v2.second,
                                                                         v_max.second, a_max.second, T_min);

    trajectory optimal_trajectory;
    optimal_trajectory.time = T_min;
    optimal_trajectory.x_seg1 = Trx.at(0);
    optimal_trajectory.x_seg2 = Trx.at(1);
    optimal_trajectory.x_seg3 = Trx.at(2);
    optimal_trajectory.y_seg1 = Try.at(0);
    optimal_trajectory.y_seg2 = Try.at(1);
    optimal_trajectory.y_seg3 = Try.at(2);
    return optimal_trajectory;
}

std::vector<std::pair<double,double>> wayPointVelocities(std::vector<std::pair<double,double>> points,
                                                         std::pair<double, double> vel_start,
                                                         std::pair<double, double> vel_finish,
                                                         double vel_max, double coeff[]) {

    unsigned long n = points.size();
    std::pair<double, double> v_max(vel_max/sqrt(2), vel_max/sqrt(2));
    std::vector<std::pair<double,double>> velocities;
    velocities.push_back(vel_start);

    std::pair<double,double> prev_point;
    std::pair<double,double> next_point;
    std::pair<double,double> point;
    double delta_x_next, delta_y_next, delta_x_prev, delta_y_prev;
    double v_x, v_y;
    double prev_length, next_length;
    double b;
    for (unsigned long i = 1; i < n-1; ++i) {
        prev_point = points[i - 1];
        next_point = points[i + 1];
        point = points[i];
        delta_x_prev = point.first - prev_point.first;
        delta_y_prev = point.second - prev_point.second;
        delta_x_next = next_point.first - point.first;
        delta_y_next = next_point.second - point.second;
        prev_length = sqrt(pow(delta_x_prev,2) + pow(delta_y_prev,2));
        next_length = sqrt(pow(delta_x_next,2) + pow(delta_y_next,2));
        v_x = (delta_x_prev/prev_length + delta_x_next/next_length)/(1/prev_length + 1/next_length);
        v_y = (delta_y_prev/prev_length + delta_y_next/prev_length)/(1/prev_length + 1/next_length);

        if (v_x > coeff[i-1] * v_max.first) {
            if (v_y > coeff[i-1] * v_max.second) {
                b = v_y/v_x;
                if (b < 1) {
                    v_x = coeff[i-1] * v_max.first;
                    v_y = b * v_x;
                } else {
                    v_y = coeff[i-1] *v_max.second;
                    v_x = v_y/b;
                }
            } else {
                b = v_y/v_x;
                v_x = coeff[i-1] *v_max.first;
                v_y = b * v_x;
            }
        } else if (v_y > coeff[i-1] *v_max.second) {
            b = v_x/v_y;
            v_y = coeff[i-1] *v_max.second;
            v_x = b * v_y;
        }
        velocities.push_back({v_x,v_y});
    }
    velocities.push_back(vel_finish);
    return velocities;
}

std::pair<std::vector<segment>,std::vector<segment>> waypointFollowing(std::vector<std::pair<double,double>> points,
                                          std::pair<double, double> vel_start,
                                          std::pair<double, double> vel_finish,
                                          double vel_max, double acc_max, unsigned int iter=1000) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> rand_coeff(0.0, 1.0);

    double bestTime;
    int best = 0;
    std::vector<std::vector<trajectory>> trajectories;
    std::vector<std::vector<std::pair<double,double>>> velocities;
    double coeff[points.size() - 2];

    for (int j = 0; j < points.size() - 2; ++j) {
        coeff[j] = 1;
    }

    std::vector<std::pair<double,double>> vel = wayPointVelocities(points, vel_start, vel_finish, vel_max, coeff);
    velocities.push_back(vel);
    double time = 0;

    std::vector<trajectory> wayPointTrajectory;
    for (int i = 0; i < points.size() - 1; ++i) {
        trajectory ct = findMinTime(points[i], points[i+1], vel[i], vel[i+1], vel_max, acc_max);
        wayPointTrajectory.push_back(ct);
        time += ct.time;
    }
    trajectories.push_back(wayPointTrajectory);
    bestTime = time;

    for (int k = 0; k < iter; ++k) {
        for (int j = 0; j < points.size() - 2; ++j) {
            coeff[j] = rand_coeff(gen);
        }

        std::vector<std::pair<double,double>> vel1 = wayPointVelocities(points, vel_start, vel_finish, vel_max, coeff);
        time = 0;
        std::vector<trajectory> wayPointTrajectory1;
        for (int i = 0; i < points.size() - 1; ++i) {
            trajectory ct = findMinTime(points[i], points[i+1], vel1[i], vel1[i+1], vel_max, acc_max);
            wayPointTrajectory1.push_back(ct);
            time += ct.time;
        }
        if (time < bestTime) {
            bestTime = time;
            best = k + 1;
        }
        velocities.push_back(vel1);
        trajectories.push_back(wayPointTrajectory1);
    }
    std::pair<std::vector<segment>,std::vector<segment>> segmentTrajectory;
    std::vector<segment> segmentsX;
    std::vector<segment> segmentsY;

    for (auto tr: trajectories[best]) {
        if (tr.x_seg1.time > 0) {
            segmentsX.push_back(tr.x_seg1);
        }
        if (tr.x_seg2.time > 0) {
            segmentsX.push_back(tr.x_seg2);
        }
        if (tr.x_seg3.time > 0) {
            segmentsX.push_back(tr.x_seg3);
        }

        if (tr.y_seg1.time > 0) {
            segmentsY.push_back(tr.y_seg1);
        }
        if (tr.y_seg2.time > 0) {
            segmentsY.push_back(tr.y_seg2);
        }
        if (tr.y_seg3.time > 0) {
            segmentsY.push_back(tr.y_seg3);
        }
    }

    segmentTrajectory.first = segmentsX;
    segmentTrajectory.second = segmentsY;

    return segmentTrajectory;
}

/*
 * TESTS
 */


void testMaxForEach() {
    std::cout<<"\ntesting maxForEach ";
    int failed = 0;

    //test
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis1(-100.0, 100.0);
    std::uniform_real_distribution<> dis2(-10.0, 10.0);
    double rand;
    std::pair<double, double> maxima;

    for (int n = 0; n < 1000; ++n) {
        rand = dis2(gen);
        maxima = maxForEach(std::pair<double, double>(dis1(gen), dis1(gen)),
                            std::pair<double, double>(dis1(gen), dis1(gen)), rand);
        if (fabs(fabs(rand) - sqrt(pow(maxima.first, 2) + pow(maxima.second, 2)))>0.00001) {
            failed += 1;
        }
        if (maxima.first < 0 || maxima.second < 0) {
            failed += 1;
        }
    }
    if (failed > 0) {
        std::cout << "\nFailed " << failed << " out of 1000\n";
    } else {
        std::cout<<"OK\n";
    }
}


void testFindMinTime() {
    std::cout<<"\ntesting findMinTime ";
    int failed = 0;
    int fail1 = 0; //total time <= 0
    int fail2 = 0; //segment time < 0
    int fail3 = 0; //|time_x - time_y| > 1e-10
    int fail4 = 0;
    int fail5 = 0; //
    int fail6 = 0; //

    //test
    double a_m = 0.1;
    double v_m = 1;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> coord(-100.0, 100.0);
    std::uniform_real_distribution<> vel(-v_m/sqrt(2), v_m/sqrt(2));

    std::pair<double, double > v1;
    std::pair<double, double > v2;
    trajectory tr;
    double time_x, time_y;
    std::pair<double, double > p1, p2;
    for (int n = 0; n < 1000; ++n) {
        p1.first = coord(gen);
        p1.second = coord(gen);
        p2.first = coord(gen);
        p2.second = coord(gen);
        v1.first = vel(gen);
        v1.second = vel(gen);
        v2.first = vel(gen);
        v2.second = vel(gen);

        tr = findMinTime(p1, p2, v1, v2, v_m, a_m);

        if (tr.time <= 0) {
            fail1 += 1;
        }

        if (tr.x_seg1.time < 0 || tr.x_seg2.time < 0 || tr.x_seg3.time < 0 ||
                tr.y_seg1.time < 0 || tr.y_seg2.time < 0 || tr.y_seg3.time < 0) {
            fail2 += 1;
        }

        time_x = tr.x_seg1.time + tr.x_seg2.time + tr.x_seg3.time;
        time_y = tr.y_seg1.time + tr.y_seg2.time + tr.y_seg3.time;
        if (fabs(time_x - time_y) > 1e-10) {
            fail3 += 1;
        }

        if (fabs(time_x - tr.time) > 1e-10 || fabs(time_y - tr.time) > 1e-10) {
            fail4 += 1;
        }

    }

    failed = (fail1 > 0) || (fail2 > 0) || (fail3 > 0) || (fail4 > 0) || (fail5 > 0);
    if (failed) {
        std::cout << "\nTest1 (total time <= 0) : " << fail1;
        std::cout << "\nTest2 (segment time < 0) : " << fail2;
        std::cout << "\nTest3 (|time_x - time_y| > 1e-10) : " << fail3;
        std::cout << "\nTest4 (time_x (or_y) != T_min) : " << fail4;
    } else {
        std::cout<<"OK\n";
    }
}


void testDMIT() {
    std::cout<<"\n\nRUNNING TESTS\n";
    testMaxForEach();
    testFindMinTime();
}
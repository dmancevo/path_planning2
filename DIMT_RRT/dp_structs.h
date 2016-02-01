//
// Created by Olga Mikheeva on 01/02/16.
//
#ifndef DP_STRUCTS_H
#define DP_STRUCTS_H

struct segment{
    double time;
    double acc;
    segment() {};
    segment(double time, double acc) : time(time), acc(acc) {};
};

struct trajectory {
    double time;
    segment x_seg1;
    segment x_seg2;
    segment x_seg3;
    segment y_seg1;
    segment y_seg2;
    segment y_seg3;
};

#endif
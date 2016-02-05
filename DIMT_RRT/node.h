//
// Created by Olga Mikheeva on 01/02/16.
//

#ifndef DIMT_RRT_NODE_H
#define DIMT_RRT_NODE_H


#include <stddef.h>

class node {
public:
    node *parent = NULL;
    double x;
    double y;
    double vx;
    double vy;
    double ax = 0; //acceleration at parent to get here
    double ay = 0; //acceleration at parent to get here
    double time = 0;
    double total_time = 0;
    double neighbours = 1;
    node(){};
};


#endif //DIMT_RRT_NODE_H

//
// Created by Olga Mikheeva on 24/01/16.
//

#include "node.h"

void node::setG(double g) {
    this->g = g;
    this->recountF();
};

void node::setH(double h) {
    this->h = h;
    this->recountF();
};

double node::getF() const {
    return this->f;
};

double node::getG() const {
    return this->g;
};

void node::recountF() {
    this->f = this->g + this->h;
};
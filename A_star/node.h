//
// Created by Olga Mikheeva on 24/01/16.
//

#include <stddef.h>

class node {
public:
    node *parent = NULL;
    int x, y;
    node() {};
    node(int x, int y) : x(x), y(y) {};
    void setG(double g);
    void setH(double h);
    double getF() const;
    double getG() const;
private:
    double f = 0, g = 0, h = 0;
    void recountF();
};

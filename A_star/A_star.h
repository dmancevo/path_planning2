//
// Created by Olga Mikheeva on 21/01/16.
//

#include <vector>
#include "node.h"

//Constants for neighbourhood
const int NB_4 = 4;
const int NB_8 = 8; // to go diagonal at least one adjacent 4-n neighbour must be available
const int NB_8_2 = 82; // to go diagonal both adjacent 4-n neighbours must be available

class A_star {
public:
    A_star(std::vector<std::vector<bool> > map);
    A_star(int n, int m);
    int n, m;
    std::vector<std::pair<int,int>> findPath(int x1, int y1, int x2, int y2, int nb);
private:
    std::vector<std::vector<bool> > map;
    double distance(node n1, node n2);
    std::vector<std::pair<int,int>> findNeighbours(node n, int NB);
};

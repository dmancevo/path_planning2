//
// Created by Olga Mikheeva on 21/01/16.
//

#include "A_star.h"
#include <cmath>
#include <set>
#include <algorithm>

using namespace std;

struct Cmp {
    bool operator ()(const node* a, const node* b) const {
        if ((a->x == b->x) && (a->y == b->y)) {
            return 0;
        } else if (a->getF() == b->getF()) {
            if (a->x != b->x) {
                return a->x < b->x;
            } else {
                return a->y < b->y;
            }
        } else {
            return a->getF() < b->getF();
        }
    }
};


struct FindCmp {
    explicit FindCmp(node* base_node) : base_node(base_node) {};

    bool operator ()(const node* a) const {
        return (a->x == base_node->x) && (a->y == base_node->y);
    }

    node* base_node;
};

A_star::A_star(int n, int m) {
    this->map = vector<vector<bool> >(n, vector<bool>(m));
    this->n = n;
    this->m = m;
};

A_star::A_star(vector<vector<bool> > map) : map(map) {
    this->n = (int) map.size();
    if (this->n > 0) {
        this->m = (int) map.at(0).size();
    } else {
        this->m = 0;
    }
};


vector<pair<int,int>> A_star::findPath(int x1, int y1, int x2, int y2, int nb) {
    vector<pair<int,int>> path;

    //if already at finish
    if (x1 == x2 && y1 == y2) {
        path.push_back(pair<int,int>(x1, y1));
        return path;
    }

    node* n_start = new node(x1, y1);
    node* n_finish = new node(x2, y2);
    n_start->setH(this->distance(*n_start, *n_finish));

    set<node*, Cmp> open_set; //set of open (not yet visited) nodes
    set<node*, Cmp> visited_set; //set of visited nodes

    //add start node to open set
    open_set.insert(n_start);

    while (!open_set.empty()) {
        node* n_current = *open_set.begin(); //take node with smallest f

        visited_set.insert(n_current);
        open_set.erase(n_current);

        //Find neighbours
        vector<pair<int,int>> nbrs = this->findNeighbours(*n_current, nb);

        for (auto const &nbr: nbrs) {
            //if found finish node, reconstruct and return path
            if (nbr.first == n_finish->x && nbr.second == n_finish->y) {
                //reconstructing path
                path.push_back(pair<int,int>(nbr.first, nbr.second)); //final node
                path.push_back(pair<int,int>(n_current->x, n_current->y)); //previous node

                node *parent = n_current->parent;
                while (parent != NULL) {
                    path.push_back(pair<int,int>(parent->x, parent->y));
                    parent = parent->parent;
                }

                reverse(path.begin(), path.end());
                return path;
            }

            //create successor node
            node* n_succ = new node(nbr.first, nbr.second);
            n_succ->parent = n_current;
            n_succ->setG(n_current->getG() + this->distance(*n_succ, *n_current));
            n_succ->setH(this->distance(*n_succ, *n_finish));

            //find position of node with same (x,y) in visited nodes
            set<node*, Cmp>::iterator visited_it = find_if(visited_set.begin(), visited_set.end(), FindCmp(n_succ));

            if (visited_it != visited_set.end()) {
                continue; //already evaluated
            }

            //find position of node with same (x,y) in open nodes
            set<node*, Cmp>::iterator open_it = find_if(open_set.begin(), open_set.end(), FindCmp(n_succ));

            if (open_it == open_set.end()) {
                //if note yet in open set, add
                open_set.insert(n_succ);
            } else {
                if (n_succ->getF() > (*open_it)->getF()) {
                    continue; //already have better version of the node in open set
                } else {
                    //if node is in open set, but new one has lower value of f, replace
                    open_set.erase(*open_it);
                    open_set.insert(n_succ);
                }
            }
        }
    }
    return path;
}

/*
 * Euclidean distance between nodes
 */
double A_star::distance(node n1, node n2) {
    return sqrt(pow(n1.x - n2.x, 2) + pow(n1.y - n2.y, 2));
}

/*
 * Finds coordinates of all valid neighbours of the node (obstacles are taken into acount)
 */
vector<pair<int,int>> A_star::findNeighbours(node n, int NB) {
    vector<pair<int,int>> nbrs;
    set<int> added;
    int dir4[4][2] = {{0,1},{1,0},{0,-1},{-1,0}};
    int dir8[4][2] = {{1,1},{1,-1},{-1,-1},{-1,1}};

    for (int i = 0; i < 4; ++i) {
        int new_x = dir4[i][0] + n.x;
        int new_y = dir4[i][1] + n.y;
        if (new_x > -1 && new_x < this->n && new_y > -1 && new_y < this->m && !this->map[new_x][new_y]) {
            nbrs.push_back(pair<int,int>(new_x, new_y));
            added.insert(i);
        }
    }

    if (NB == NB_8 || NB == NB_8_2) {
        for (int i = 0; i < 4; ++i) {
            int new_x = dir8[i][0] + n.x;
            int new_y = dir8[i][1] + n.y;
            if (new_x > -1 && new_x < this->n && new_y > -1 && new_y < this->m && !this->map[new_x][new_y]) {
                if (NB == NB_8) {
                    if (added.count(i) || ((i == 3) ? added.count(0) : added.count(i + 1))) {
                        nbrs.push_back(pair<int, int>(new_x, new_y));
                    }
                } else {
                    if (added.count(i) && ((i == 3) ? added.count(0) : added.count(i + 1))) {
                        nbrs.push_back(pair<int, int>(new_x, new_y));
                    }
                }
            }
        }
    }

    return nbrs;
}


//
// Created by Olga Mikheeva on 01/02/16.
//

#include "DPointPathPlanner.h"
#include <random>
#include <iostream>


DPointPathPlanner::DPointPathPlanner(double x_min, double x_max, double y_min, double y_max,
                                     double acc_max, double vel_max) {

    this->x_min = x_min;
    this->x_max = x_max;
    this->y_min = y_min;
    this->y_max = y_max;
    this->acc_max = acc_max;
    this->vel_max = vel_max;
}

std::pair<std::vector<segment>,std::vector<segment>> DPointPathPlanner::findPath(
        std::pair<double, double> start_coord,
        std::pair<double, double> finish_coord,
        std::pair<double, double> start_vel,
        std::pair<double, double> finish_vel) {

    std::pair<std::vector<segment>,std::vector<segment>> path;
    std::vector<segment> path_x;
    std::vector<segment> path_y;

    //this->fs.open ("test.txt", std::fstream::in | std::fstream::out | std::fstream::app);
    //this->fs2.open ("path.txt", std::fstream::in | std::fstream::out | std::fstream::app);

    this->start_coord = start_coord;
    this->finish_coord = finish_coord;
    this->start_vel = start_vel;
    this->finish_vel = finish_vel;

    //create root node
    node* root_node = new node();
    root_node->x = this->start_coord.first;
    root_node->y = this->start_coord.second;
    root_node->vx = this->start_vel.first;
    root_node->vy = this->start_vel.second;

    this->nodes.push_back(root_node);
    this->weights.push_back(1);



    int graph_size = 1;

    while(true) {
        int random_node_ID = this->getRandomNodeID();
        std::pair<double, double> acc = this->getRandomAcc();
        double time = this->getRandomTime();
        node *new_node = this->propagate(this->nodes[random_node_ID], acc, time);
        if (new_node != NULL) {
            this->nodes.push_back(new_node);
            //this->fs<<new_node->x<<" "<<new_node->y<<"\n";
            graph_size += 1;
            //TODO check if in endgame region, success (add final piece)

            double dist = sqrt(pow(new_node->x - this->finish_coord.first,2)
                               + pow(new_node->y - this->finish_coord.second,2));
            if (dist < this->endgame_region) {
                std::cout<<"\nIn endgame region ("<<new_node->x<<", "<<new_node->y<<")";

                std::cout<<"\n"<<graph_size;

                std::cout<<"\nTime = "<<new_node->total_time;

                //path_x.push_back(segment(new_node->time, new_node->ax));
                //this->fs2<<new_node->x<<" "<<new_node->y<<"\n";
                //node *parent = new_node->parent;
                /*
                while (parent->parent != NULL) {
                    //path_x.push_back(segment(parent->time, parent->ax));
                    //this->fs2<<parent->x<<" "<<parent->y<<"\n";
                    parent = parent->parent;
                }
                 */
                //this->fs2<<parent->x<<" "<<parent->y<<"\n\n";
                //return path;
            }
            else {
                //std::cout<<"\n"<<dist;
            }


            if (graph_size%1000 == 0) {
                std::cout<<"\n"<<graph_size;
            }

            if (graph_size > this->node_limit) {
                std::cout<<"\nFailure";
                //this->fs.close();
                //this->fs2.close();
                return path; //failure - empty trajectory
            }

            //TODO update weights
            this->updateWeights(new_node);
            //this->weights.push_back(1);
        }
    }

}



int DPointPathPlanner::getRandomNodeID() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::discrete_distribution<> d(this->weights.begin(), this->weights.end());
    return d(gen);
}

std::pair<double, double> DPointPathPlanner::getRandomAcc() {
    std::pair<double, double> control;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> randAcc1(-this->acc_max, this->acc_max);
    control.first = randAcc1(gen);
    double restr = sqrt(pow(this->acc_max,2) + pow(control.first,2));
    std::uniform_real_distribution<> randAcc2(-restr, restr);
    control.second = randAcc2(gen);
    /*
    bool inLimit = false;
    while(!inLimit) {
        control.first = randAcc(gen);
        control.second = randAcc(gen);
        if(sqrt(pow(control.first, 2) + pow(control.second, 2)) <= this->acc_max) {
            inLimit = true;
        }
    }*/
    return control;
}

double DPointPathPlanner::getRandomTime() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> time(this->delta_time_min, this->delta_time_max);
    return time(gen);
}

node * DPointPathPlanner::propagate(node *parent, std::pair<double, double> acc, double time) {
    //compute parameters for new node
    node* new_node = new node();
    new_node->parent = parent;
    new_node->ax = acc.first;
    new_node->ay = acc.second;
    new_node->time = time;
    new_node->total_time = parent->total_time + new_node->time;

    //compute velocities
    new_node->vx = parent->vx + new_node->ax * new_node->time;
    new_node->vy = parent->vy + new_node->ay * new_node->time;

    //check velocity limits
    if (sqrt(pow(new_node->vx, 2) + pow(new_node->vy, 2)) > this->acc_max) {
        return nullptr;
    }

    //compute coordinates at the end
    new_node->x = 0.5 * new_node->ax * pow(new_node->time, 2) + parent->vx * new_node->time + parent->x;
    new_node->y = 0.5 * new_node->ay * pow(new_node->time, 2) + parent->vy * new_node->time + parent->y;

    //check trajectory for coordinate limits and obstacles

    bool valid_trajectory = this->isValidTrajectory(parent, new_node);

    if (!valid_trajectory) {
        return nullptr;
    }

    return new_node;
}

bool DPointPathPlanner::isValidTrajectory(node *parent, node *new_node) {
    double time = this->delta_tr_check;
    double x, y;
    //check trajectory
    while (time < new_node->time) {
        //compute coordinates
        x = 0.5 * new_node->ax * pow(time, 2) + parent->vx * time + parent->x;
        y = 0.5 * new_node->ay * pow(time, 2) + parent->vy * time + parent->y;

        //check if coordinates are in the range
        if (x < this->x_min || x > this->x_max || y < this->y_min || y > this->y_max) {
            return false;
        }
        //TODO check obstacles
        time += this->delta_tr_check;
    }
    //check end coordinates
    if (new_node->x < this->x_min || new_node->x > this->x_max
        || new_node->y < this->y_min || new_node->y > this->y_max) {
        return false;
    }
    //TODO check obstacles for the end coordinates
    return true;
}

void DPointPathPlanner::updateWeights(node *new_node) {

    double dist;
    unsigned long n = this->nodes.size();
    for (int i = 0; i < n - 1; ++i) {
        node *old_node = this->nodes[i];
        //check distance between those nodes
        dist = sqrt(pow(new_node->x - old_node->x,2) + pow(new_node->y - old_node->y,2));

        if (dist <= this->neighbourhood) {
            old_node->neighbours += 1;
            this->weights[i] = 1/old_node->neighbours;

            new_node->neighbours += 1;
        }
    }
    this->weights.push_back(1/new_node->neighbours);
}

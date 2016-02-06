#include <utility>
#include <iostream>
#include <algorithm> 
#include <vector>
#include <limits>
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <cmath>
#include "V_graph.h"


std::vector<std::pair<double,double> > V_graph::shortest_path(){

	std::vector<int> prev = this->Dijkstra();

	int v=nodeCoords.size()-1;
	std::vector<std::pair<double,double> > shortest;
	while(v!=0){
		shortest.push_back(nodeCoords[v]);
		v = prev[v];
	}
	shortest.push_back(nodeCoords[v]);

	return shortest;
};

int min_dist(std::vector<int> *Q,std::vector<double> dist){
	double min = std::numeric_limits<double>::infinity();
	int ind, u;
	for (int v=0;v<Q->size();v+=1){
		if (dist[Q->at(v)] < min){
			min = dist[Q->at(v)];
			ind = v;
		}
	}

	u = Q->at(ind);
	Q->erase(Q->begin() + ind);
	return u;
}

std::vector<int> V_graph::Dijkstra(){

	//Infinity
	double inf = std::numeric_limits<double>::infinity();

	//Node index, node disance from start
	std::vector<int> Q;

	// //Distance and previous node in optimal path
	std::vector<double> dist;
	std::vector<int> prev;

	// Initialization
	for (int v=0;v<nodeCoords.size();v+=1){
		dist.push_back(inf);
		prev.push_back(-1);
		Q.push_back(v);
	}

	//Distance from start
	dist[0] = 0;

	int u;
	double alt, length;
	while(!Q.empty()){

		u = min_dist(&Q,dist);

		for(int v=0;v<adjList[u].size();v+=1){
			length = sqrt(pow(nodeCoords[u].first-nodeCoords[adjList[u][v]].first,2) +
				pow(nodeCoords[u].second-nodeCoords[adjList[u][v]].second,2));

			alt = dist[u] + length;

			if(alt<dist[adjList[u][v]]){
					dist[adjList[u][v]] = alt;
					prev[adjList[u][v]] = u;
				}
		}

	}

	return prev;
};

V_graph::V_graph(std::string polyObstMap, double eta){

	//Increase eta
	eta *= 1000;

	//Load polygons
	this->loadPolygons(polyObstMap);

	//Enlage polygon
	this->enlarge(eta);

	//Track keeping
	int map[40][20];
	int m = 0;

	//Node coordinates
	nodeCoords.push_back(start);
	for (int k=0;k<polygons.size();k+=1){
		for (int a=0;a<polygons[k].size();a+=1){
			nodeCoords.push_back(graph_polygons[k][a]);
			m+=1;
			map[k][a] = m;
		}
	}
	nodeCoords.push_back(end);

	//Adjacency list
	std::vector<int> *neighbors;
	for (int i=0;i<nodeCoords.size();i+=1){
		neighbors = new std::vector<int>();
		adjList.push_back(*neighbors);
	}

	//Connect edges
	for (int i=0;i<nodeCoords.size();i+=1){
		for (int j=0;j<nodeCoords.size();j+=1){
			if (i==j)
				continue;
			if(this->validPath(i,j)==0)
				adjList[i].push_back(j);

		}
	}

};

//Determine if there is a polygon between a and b
//If so return 1 else 0
//reference: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Mathematics
int V_graph::validPath(int a, int b){

	std::pair<double, double> a_point, b_point;

	//Check each polygon
	for (int i=0; i<polygons.size(); i+=1){
		for (int j=0; j<polygons[i].size(); j+=1){

			a_point = std::make_pair(nodeCoords[a].first,nodeCoords[a].second);
			b_point = std::make_pair(nodeCoords[b].first,nodeCoords[b].second);

			if (this->intersect(a_point,b_point,i,j,true)==1)
				return 1;
		}
	}

	return 0;
};

//a_point and b_point: coordinates of two points in the map.
//return 1 if there is a polygon between them else 0.
int V_graph::validPath(std::pair<double, double> a_point, std::pair<double, double> b_point){
		//Check each polygon
	for (int i=0; i<polygons.size(); i+=1){
		for (int j=0; j<polygons[i].size(); j+=1){

			if (this->intersect(a_point,b_point,i,j,true)==1)
				return 1;
		}
	}

	return 0;
}

int V_graph::intersect(std::pair<double,double> a_point, std::pair<double,double> b_point, int i, int j, bool e){

	std::pair<double,double> p, *n1, *n2;
	double x1,y1,x2,y2,x3,y3,x4,y4,den,px,py,alpha1,alpha2;

	if (e){
		n1 = &polygons[i][j];
		if(j+1 < polygons[i].size()){
			n2 = &polygons[i][j+1];
		} else{
			n2 = &polygons[i][0];
		}
	} else{
		n1 = &graph_polygons[i][j];
		if(j+1 < graph_polygons[i].size()){
			n2 = &graph_polygons[i][j+1];
		} else{
			n2 = &graph_polygons[i][0];
		}
	}

	x1 = a_point.first;
	y1 = a_point.second;
	x2 = b_point.first;
	y2 = b_point.second;

	x3 = n1->first;
	y3 = n1->second;
	x4 = n2->first;
	y4 = n2->second;

	den = ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));

	//Check if lines are parallel
	if (den==0)
		return 0;

	//Determine point of intersection
	px = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4) )/den;
	py = ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4))/den;

	p = std::make_pair(px,py);

	//Bezier parameters
	alpha1 = (px-x2)/(x1-x2);
	alpha2 = (px-x4)/(x3-x4);

	if(alpha1 < 1 && alpha1 > 0
		&& alpha2 < 1 && alpha2 > 0)
		return 1;
	else
		return 0;

};

bool V_graph::in_polygon(std::pair<double,double> point){

	std::pair<double, double> ray_origin = std::make_pair(-1,-1);
	int num_intersections = 0;

	//Check each polygon
	for (int i=0; i<polygons.size(); i+=1){
		for (int j=0; j<polygons[i].size(); j+=1){

			num_intersections += this->intersect(point,ray_origin,i,j,true);

		}
	}

	if(num_intersections%2==0)
		return false;
	else
		return true;

};

void V_graph::enlarge(double eta){

	//Translate polygon nodes
	double alpha,beta;
	std::pair<double,double> *left, *node, *right, p1, p2, t_center;
	for(int k=0;k<polygons.size();k++){
		for(int a=0;a<polygons[k].size();a++){

			node = &polygons[k][a];

			//Get the nodes to the left and right of my node
			if (a==0)
				left = &polygons[k][polygons[k].size()-1];
			else
				left = &polygons[k][a-1];

			if (a+1<polygons[k].size())
				right = &polygons[k][a+1];
			else
				right = &polygons[k][0];

			//Compute center of triangle
			alpha = 1.0 - 0.001/(sqrt(pow(node->first-left->first,2)+
				pow(node->second-left->second,2)));

			beta = 1.0 - 0.001/(sqrt(pow(node->first-right->first,2)+
				pow(node->second-right->second,2)));

			p1 = std::make_pair((1-alpha)*left->first + alpha*node->first,
				(1-alpha)*left->second + alpha*node->second);

			p2 = std::make_pair((1-beta)*right->first + beta*node->first,
				(1-beta)*right->second + beta*node->second);

			t_center = std::make_pair((p1.first+node->first+p2.first)/3.0,
				(p1.second+node->second+p2.second)/3.0);

			// std::cout<<"np.array(["<<this->in_polygon(t_center)<<", "<<t_center.first<<", "<<t_center.second<<"]),\n";

			//Translate node as appropiate
			if (this->in_polygon(t_center)){
				graph_polygons[k][a].first = polygons[k][a].first * (1+eta) +
					t_center.first * -eta;

				graph_polygons[k][a].second = polygons[k][a].second * (1+eta) +
					t_center.second * -eta;

			} else{

				graph_polygons[k][a].first = t_center.first * (1+eta) +
					polygons[k][a].first * -eta;

				graph_polygons[k][a].second = t_center.second * (1+eta) +
					polygons[k][a].second * -eta;
			}

		}
	}

};

//Load polygons, start and end coords from txt file
void V_graph::loadPolygons(std::string polyObstMap){

	std::ifstream file(polyObstMap.c_str());
	std::string str;
	std::vector<std::pair<double,double> > *polygon = new std::vector<std::pair<double,double> >();
	std::pair<double,double> point;

	int i = 0;
	while (std::getline(file, str))
    {
        std::vector<std::string> coords;
        boost::split(coords, str, boost::is_any_of(" "), boost::token_compress_on);

        point = std::make_pair(atof(coords[0].c_str()),
        	atof(coords[1].c_str()));

        if (i == 0)
        	start = point;

        if (i==1)
        	end = point;

        if (i > 2){

        	if (point.first == 0.0 && point.second == 0.0){
        		polygons.push_back(*polygon);
        		graph_polygons.push_back(*polygon);
        		polygon = new std::vector<std::pair<double,double> >();
        	} else{
        		polygon->push_back(point);
        	}
        	
        }
        i+=1;
    }

};
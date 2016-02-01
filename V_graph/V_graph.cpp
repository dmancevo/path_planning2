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

	//Load polygons
	this->loadPolygons(polyObstMap);

	//Enlage polygon
	this->enlarge(eta);

	//Track keeping
	int map[20][20];
	int rmap[40];
	int m = 0;

	//Node coordinates
	nodeCoords.push_back(start);
	rmap[0] = -1;
	for (int k=0;k<polygons.size();k+=1){
		for (int a=0;a<polygons[k].size();a+=1){
			nodeCoords.push_back(graph_polygons[k][a]);
			m+=1;
			map[k][a] = m;
			rmap[m] = k;
		}
	}
	nodeCoords.push_back(end);

	//Adjacency list
	std::vector<int> *neighbors;
	for (int i=0;i<nodeCoords.size();i+=1){
		neighbors = new std::vector<int>();
		adjList.push_back(*neighbors);
	}

	//Connect edges belonging to the same polygon
	//by connecting the nodes that form the convex hull
	int b, ma, mb;
	for (int k=0;k<polygons.size();k+=1){
		for(int a=0;a<polygons[k].size();a+=1){

			if (a+1<polygons[k].size())
				b = a+1;
			else
				b = 0;

			ma = map[k][a];
			mb = map[k][b];

			if(this->O(k,a,b)==0 && O1(ma,mb)==0){
				adjList[ma].push_back(mb);
				adjList[mb].push_back(ma);
			}
		}
	}

	//Connect edges belonging to different polygons
	int k1, k2;
	for (int i=0;i<nodeCoords.size();i+=1){
		for (int j=0;j<nodeCoords.size();j+=1){
			k1 = rmap[i];
			k2 = rmap[j];
			if (k1==k2)
				continue;
			if(O1(i,j)==0)
				adjList[i].push_back(j);

		}
	}

};

//Determine if a straight line between a and b,
//where a and b belong to the same polygon,
//cuts through the interior of such polygon.
//If so return 1 else 0
int V_graph::O(int k, int a, int b){

	std::pair<double,double> *n1,*n2, w;
	double w0, wDot;
	int above, below;

	//Line between n1 and n2
	n1 = &polygons[k][a];
	n2 = &polygons[k][b];

	//Separating hyperplane
	w = std::make_pair(n2->second-n1->second,n1->first-n2->first);
	w0 = -(w.first*n1->first+w.second*n1->second);

	//Check if line cuts through the polygon
	above = 0;
	below = 0;
	for (int i=0;i<polygons[k].size();i+=1){
		if(i==a || i==b)
			continue;

		wDot = (w.first*polygons[k][i].first +
			w.second*polygons[k][i].second) + w0;

		if (wDot > 0)
			above += 1;
		else if (wDot < 0)
			below += 1;

		if (above > 0 && below > 0)
			return 1;

	}

	return 0;

};

//Determine if there is a polygon between a and b
//If so return 1 else 0
//reference: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Mathematics
int V_graph::O1(int a, int b){

	//Check each polygon
	std::pair<double,double> p, *n1, *n2;
	double x1,y1,x2,y2,x3,y3,x4,y4,den,px,py,alpha1,alpha2;
	for (int i=0; i<polygons.size(); i+=1){
		for (int j=0; j<polygons[i].size(); j+=1){

			n1 = &polygons[i][j];

			if(j+1 < polygons[i].size()){
				n2 = &polygons[i][j+1];
			} else{
				n2 = &polygons[i][0];
			}

			x1 = nodeCoords[a].first;
			y1 = nodeCoords[a].second;
			x2 = nodeCoords[b].first;
			y2 = nodeCoords[b].second;
			x3 = n1->first;
			y3 = n1->second;
			x4 = n2->first;
			y4 = n2->second;

			den = ((x1-x2)*(y3-y4) - (y1-y2)*(x3-x4));

			//Check if lines are parallel
			if (den==0)
				continue;

			//Determine point of intersection
			px = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4) )/den;
			py = ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4))/den;

			p = std::make_pair(px,py);

			//Bezier parameters
			alpha1 = (px-x2)/(x1-x2);
			alpha2 = (px-x4)/(x3-x4);

			if(alpha1 < 0.999 && alpha1 > 0.001
				&& alpha2 < 0.999 && alpha2 > 0.001)
				return 1;

		}
	}
	return 0;
};

void V_graph::enlarge(double eta){

	//Compute polygon centers
	std::vector<std::pair<double,double> > centers;
	double cx, cy;
	for (int k=0;k<polygons.size();k+=1){

		cx=0;
		cy=0;
		for (int a=0;a<polygons[k].size();a+=1){
			cx += polygons[k][a].first;
			cy += polygons[k][a].second;
		}
		cx = cx/polygons[k].size();
		cy = cy/polygons[k].size();
		centers.push_back(std::make_pair(cx,cy));
	}

	//Translate polygon nodes
	for (int k=0;k<polygons.size();k+=1){
		for (int a=0;a<polygons[k].size();a+=1){
			graph_polygons[k][a].first = polygons[k][a].first * (1+eta) +
				centers[k].first * -eta;

			graph_polygons[k][a].second = polygons[k][a].second * (1+eta) +
				centers[k].second * -eta;
		}
	}

	//Print centers
	// for (int k=0;k<polygons.size();k+=1)
	// 	std::cout<<"np.array(["<<centers[k].first<<", "<<centers[k].second<<"]),\n";

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
#include <utility>
#include <iostream>
#include <vector>
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <cmath>
#include "V_graph.h"

V_graph::V_graph(std::string polyObstMap){

	//Load polygons
	this->loadPolygons(polyObstMap);

	//Build graph
	std::cout<<this->closestNode(start,1)<<"\n";

};

//Return index of closest node belonging to polygon i from myNode
int V_graph::closestNode(std::pair<double,double> myNode, int i){

	int closest = 0;
	double d = pow(myNode.first-polygons[i][0].first,2) + 
			pow(myNode.second-polygons[i][0].second,2);

	double new_d;
	for (int j=1;j<polygons[i].size();j+=1){

		new_d = pow(myNode.first-polygons[i][j].first,2) + 
			pow(myNode.second-polygons[i][j].second,2);

		if (new_d < d){
			closest = j;
			d = new_d;
		}
	}

	return closest;
};

int V_graph::obstacles(std::pair<double,double> a, std::pair<double,double> b){

	//Separating plane
	std::pair<double,double> w = std::make_pair(b.second-a.second,a.first-b.first);
	double w0 = -(w.first*a.first + w.second*a.second);

	//Check each polygon
	int above;
	int below;
	for (int i=0; i<polygons.size(); i+=1){
		above = 0;
		below = 0;
		for (int j=0; j<polygons[i].size(); j+=1){

			if (w.first*polygons[i][j].first +
				w.second*polygons[i][j].second + w0 > 0){
				below += 1;
			} else{
				above += 1;
			}

			if (above > 0 and below > 0)
				return i;
		}
	}

	return -1;
};

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
        		polygon = new std::vector<std::pair<double,double> >();
        	} else{
        		polygon->push_back(point);
        	}
        	
        }
        i+=1;
    }

};


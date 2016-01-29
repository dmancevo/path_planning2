#include <utility>
#include <iostream>
#include <vector>
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include "V_graph.h"

struct Point{int x; int y;};

V_graph::V_graph(std::string polyObstMap){
	polygons = readPolygonMap(polyObstMap);
};


bool V_graph::borders(double x, double y){

};

std::vector<std::pair<double,double> > readPolygonMap(std::string polyObstMap){
	std::ifstream file(polyObstMap.c_str());
	std::string str;
	std::pair<double,double> point;
	std::vector<std::pair<double,double> > polygons;

	while (std::getline(file, str))
    {
        std::vector<std::string> coords;
        boost::split(coords, str, boost::is_any_of(" "), boost::token_compress_on);

        point = std::make_pair(atof(coords[0].c_str()),
        	atof(coords[1].c_str()));

        polygons.push_back(point);
    }
    return polygons;
};


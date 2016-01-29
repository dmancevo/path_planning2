#include <utility>
#include <iostream>
#include <vector>
#include <fstream>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include "V_graph.h"

struct Point{int x; int y;};

V_graph::V_graph(std::string polyObstMap){
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


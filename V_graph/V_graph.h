#include <utility>
#include <iostream>
#include <vector>


class V_graph{

public:
	V_graph(std::string polyObstMap);
	std::vector<std::pair<double,double> > nodes;
	std::pair<double,double> start;
	std::pair<double,double> end;


private:
	std::vector<std::vector<std::pair<double,double> > >polygons;
};
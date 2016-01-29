#include <utility>
#include <iostream>
#include <vector>


class V_graph{

public:
	V_graph(std::string polyObstMap);
	std::vector<std::pair<double,double> > nodes;

private:
	std::vector<std::pair<double,double> > polygons;
	bool borders(double x, double y);
};
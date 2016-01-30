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
	void loadPolygons(std::string polyObstMap);
	int obstacles(std::pair<double,double> a, std::pair<double,double> b);
	int closestNode(std::pair<double,double> myNode, int i);
};
#include <utility>
#include <iostream>
#include <vector>


class V_graph{

public:
	V_graph(std::string polyObstMap);
	std::vector<std::pair<double,double> > nodeCoords;
	std::vector<std::vector<int> > adjList;
	void npEdges();
private:
	std::vector<std::vector<std::pair<double,double> > >polygons;
	void loadPolygons(std::string polyObstMap);
	int O(int k, int a, int b);
	int O1(int a, int b);
	std::pair<double,double> start;
	std::pair<double,double> end;
};
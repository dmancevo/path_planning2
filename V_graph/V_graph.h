#include <utility>
#include <iostream>
#include <vector>


class V_graph{

public:
	V_graph(std::string polyObstMap,double eta);
	std::vector<std::pair<double,double> > nodeCoords;
	std::vector<std::vector<int> > adjList;
	std::vector<std::vector<std::pair<double,double> > >polygons;
	std::vector<std::pair<double,double> > shortest_path();
	std::pair<double,double> start;
	std::pair<double,double> end;
private:
	std::vector<std::vector<std::pair<double,double> > >graph_polygons;
	void loadPolygons(std::string polyObstMap);
	void enlarge(double eta);
	int validPath(int a, int b);
	int intersect(std::pair<double, double> a_point, std::pair<double, double> b_point, int i, int j, bool e);
	bool in_polygon(std::pair<double,double> point);
	std::vector<int> Dijkstra();
};
#ifndef DIJKSTRA_
#define DIJKSTRA_
#include<bits/stdc++.h>

#include "priority_queue.h"
#include "LoadGraphfromFile.h"

using namespace BGL_DEFINITIONS;

double INF_VAL = std::numeric_limits<double>::max();
namespace po = boost::program_options;

class Dijkstra
{
public:

	std::unordered_map<Vertex,bool> sptSet; //shortestPath set
	std::unordered_map<Vertex,Vertex> parentMap;
	std::unordered_map<Vertex,double> distanceMap;

	Vertex start;
	Vertex goal;

	Vertex minDistance(Graph &g);
	double getDistance(Graph &g, Vertex & u, Vertex &v);
	std::vector <Vertex> getSuccessors(Graph &g, Vertex &v);
	std::vector<Vertex> findPath(Graph &g,Vertex & start, Vertex & goal, double &cost);


public:
	Dijkstra(){};
	std::vector<Vertex> computeShortestPath(Graph &g, Vertex & start, Vertex & end, double &cost);
};

#endif

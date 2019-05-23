#ifndef LPA_STAR_
#define LPA_STAR_
#include<bits/stdc++.h>

#include "priority_queue.h"
#include "LoadGraphfromFile.h"

using namespace std;

using namespace BGL_DEFINITIONS;

double INF_VAL = std::numeric_limits<double>::max();
namespace po = boost::program_options;

class LPAStar
{
private:
	Vertex start;
	Vertex goal;
	priorityQueue pq;

	std::unordered_map<int, Vertex> nodeMap; //because we are using listS


	void insertPQ(Graph &g, Vertex &node, pair<double,double> priority);
	void removePQ(Graph &g, Vertex &node);
	bool isemptyPQ();
	bool containsPQ(Graph &g, Vertex &node);
	pair <double,double>  topKeyPQ();
	Vertex popPQ(Graph &g);


	pair<double,double> calculateKey(Graph &g, Vertex & node);
	double getHeuristic(Graph &g, Vertex &u, Vertex &v);
	std::vector <Vertex> getPredecessors(Graph &g, Vertex &v);
	std::vector <Vertex> getSuccessors(Graph &g, Vertex &v);
	std::vector <Vertex> findPath(Graph &g, double & cost);

public:
	LPAStar(){}
	void initialize(Graph &g, Vertex &_start, Vertex &_goal);
	void updateNode(Graph &g, Vertex &node);
	std::vector<Vertex> computeShortestPath(Graph &g, double & cost);
};

#endif
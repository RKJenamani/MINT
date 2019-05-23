#include "LPAStar.h"

void LPAStar::insertPQ(Graph &g, Vertex &node, pair<double,double> priority)
{
	VPIndexMap index_map = get(&VProp::vertex_index, g);
	pq.insert(index_map[node],priority.first,priority.second);
	nodeMap[index_map[node]] = node;
}
void LPAStar::removePQ(Graph &g, Vertex &node)
{
	VPIndexMap index_map = get(&VProp::vertex_index, g);
	pq.remove(index_map[node]);
}
bool LPAStar::isemptyPQ()
{
	return pq.PQsize()==0;
}
bool LPAStar::containsPQ(Graph &g, Vertex &node)
{
	VPIndexMap index_map = get(&VProp::vertex_index, g);
	return pq.contains(index_map[node]);
}
pair <double,double> LPAStar::topKeyPQ()
{
	return pq.topKey();
}
Vertex LPAStar::popPQ(Graph &g)
{
	int top_index = pq.pop();
	return nodeMap[top_index];
}

void LPAStar::initialize(Graph & g, Vertex & _start, Vertex & _goal)
{
	pq.reset();
	nodeMap.clear();
	
	start=_start;
	goal=_goal;
	VertexIter vi,viend;
	for( boost::tie(vi,viend) = vertices(g); vi!=viend;++vi)
	{
		put(&VProp::gValue,g,*vi,INF_VAL);
		put(&VProp::rhsValue,g,*vi,INF_VAL);
	}
	put(&VProp::rhsValue,g,start,0);
	insertPQ(g,start,calculateKey(g,start));
}

std::vector<Vertex> LPAStar::computeShortestPath(Graph &g, double &cost)
{
	VPrhsMap rhsMap = get(&VProp::rhsValue,g);
	VPgMap gMap = get(&VProp::gValue,g);
	VPIndexMap index_map = get(&VProp::vertex_index, g);
	// std::cout<<(topKeyPQ()<calculateKey(g,goal))<<" "<< (rhsMap[goal] != gMap[goal])<<std::endl;
	while(topKeyPQ()<calculateKey(g,goal) || get(&VProp::rhsValue,g,goal) != get(&VProp::gValue,g,goal) )
	{
		Vertex node = popPQ(g);

		// pq.printPQ();
		// std::cout<<"Node:"<<index_map[node]<<std::endl;
		if( get(&VProp::rhsValue,g,node) < get(&VProp::gValue,g,node))
		{
			put(&VProp::gValue,g,node,rhsMap[node]);
			std::vector<Vertex> successors = getSuccessors(g,node);
			for (auto successor : successors)
			{
				// std::cout<<index_map[successor]<<"<-if ";
				updateNode(g,successor);
			}
		}
		else
		{
			put(&VProp::gValue,g,node,INF_VAL);
			updateNode(g,node);
			std::vector<Vertex> successors = getSuccessors(g,node);
			for (auto successor : successors )
			{
				// std::cout<<index_map[successor]<<"<-else ";
				updateNode(g,successor);
			}
		}
		// pq.printPQ();
		// std::cout<<std::endl;;
	}

	return findPath(g,cost);

	// return std::vector< Vertex>();
}

void LPAStar::updateNode(Graph &g, Vertex & node)
{
	VPrhsMap rhsMap = get(&VProp::rhsValue,g);
	VPgMap gMap = get(&VProp::gValue,g);
	EPWeightMap weightMap = get(&EProp::weight,g); 		
	if(node!=start)
	{
		std::vector<Vertex> predecessors = getPredecessors(g,node);
		double min_rhs = INF_VAL;
		for (auto predecessor : predecessors )
		{
			Edge currEdge = boost::edge(predecessor,node,g).first;
			min_rhs=min(min_rhs,gMap[predecessor]+weightMap[currEdge]);
		}
		// std::cout<<std::endl<<min_rhs<<std::endl;
		put(&VProp::rhsValue,g,node,min_rhs);
		if(containsPQ(g,node))
			removePQ(g,node);
		if(get(&VProp::rhsValue,g,node) != get(&VProp::gValue,g,node))
		{
			// std::cout<<"inserted"<<std::endl;
			insertPQ(g,node,calculateKey(g,node));
		}
	}
}

double LPAStar::getHeuristic(Graph &g, Vertex & u, Vertex &v)
{
	VPStateMap stateMap = get(&VProp::state,g);
	return (stateMap[u]-stateMap[v]).norm();
}

std::vector <Vertex> LPAStar::getPredecessors(Graph &g, Vertex &v)
{
	std::vector<Vertex> predecessors;
	InEdgeIter ei, ei_end;

	for (tie(ei, ei_end) = in_edges(v, g); ei != ei_end; ++ei) 
	{
    	Vertex curPred = source(*ei, g);
    	predecessors.push_back(curPred);
	}

  return predecessors;
}
std::vector <Vertex> LPAStar::getSuccessors(Graph &g, Vertex &v)
{
	std::vector<Vertex> successors;
	OutEdgeIter ei, ei_end;

	for (tie(ei, ei_end) = out_edges(v, g); ei != ei_end; ++ei) 
	{
    	Vertex curPred = target(*ei, g);
    	successors.push_back(curPred);
	}

  return successors;
}



pair<double,double> LPAStar::calculateKey(Graph &g, Vertex &node)
{
	VPrhsMap rhsMap = get(&VProp::rhsValue,g);
	VPgMap gMap = get(&VProp::gValue,g);

	pair <double,double> p;
	p.first = min(rhsMap[node],gMap[node]) + getHeuristic(g,node,goal); //add L2 norm here!!!
	p.second = min(rhsMap[node],gMap[node]);
	return p;
}

std::vector <Vertex> LPAStar::findPath(Graph &g, double &cost)
{	
	VPgMap gMap = get(&VProp::gValue,g);
	VPrhsMap rhsMap = get(&VProp::rhsValue,g);
	EPWeightMap weightMap = get(&EProp::weight,g);
	if (rhsMap[goal]==INF_VAL)
		return std::vector< Vertex>();

	std::vector<Vertex> finalPath;

	finalPath.push_back(goal);

	Vertex node = goal;
	Vertex new_node;
	cost=0;
	while(node!=start)
	{	
		std::vector<Vertex> predecessors = getPredecessors(g,node);
		double min_val = INF_VAL;
		double min_weight;
		for (auto predecessor : predecessors )
		{
			Edge currEdge = boost::edge(predecessor,node,g).first;
			if(min_val>gMap[predecessor]+weightMap[currEdge])
			{
				min_val=gMap[predecessor]+weightMap[currEdge];
				min_weight=weightMap[currEdge];
				new_node = predecessor;
			}
		}
		cost+=min_weight;
		finalPath.push_back(new_node);
		node=new_node;
	}
	std::reverse(finalPath.begin(), finalPath.end());

	return finalPath;
}
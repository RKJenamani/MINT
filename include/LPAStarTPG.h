#ifndef LPA_STAR_TPG_
#define LPA_STAR_TPG_



#include "tensorProductGenerator.h"
#include "heap_indexed.h"
#include "BGLDefinitions.h"
#include "LoadGraphfromFile.h" //change before commiting to repo

using namespace pr_bgl;
using namespace BGL_DEFINITIONS;

double INF_VAL = std::numeric_limits<double>::max();




class LPAStarTPG {

	heap_indexed<double> mPQ;

	Vertex mStartNode;
	Vertex mGoalNode;

	std::unordered_map<Vertex, double> mDistance;
	std::unordered_map<Vertex, double> mDistanceLookahead;

	std::unordered_map<Vertex, Vertex> mPrev;
	std::unordered_map<int, Vertex> mIndexToNode;

	Graph left_map;
	Graph right_map;

	TensorPG tpg;
	
	double getDistance(Vertex &v) 
	{ 
		return mDistance[v]; 
	}

	double getDistanceLookahead(Vertex &v) 
	{ 
		return mDistanceLookahead[v]; 
	}

	void updatePQ(Graph &g, Vertex &v, double newPriority) 
	{
		VPIndexMap index_map = get(&VProp::vertex_index, g);
		mPQ.update(index_map[v], newPriority);
	}

	void insertPQ(Graph &g, Vertex &v, double priority) 
	{
		VPIndexMap index_map = get(&VProp::vertex_index, g);
		size_t node_index = index_map[v];
		mPQ.insert(node_index, priority);
		mIndexToNode[node_index] = v;
	}

	void removePQ(Graph &g, Vertex &v) 
	{
		VPIndexMap index_map = get(&VProp::vertex_index, g);
		mPQ.remove(index_map[v]);
	}

	bool containsPQ(Graph &g, Vertex &v) 
	{
		VPIndexMap index_map = get(&VProp::vertex_index, g);
		return mPQ.contains(index_map[v]);
	}

	bool isEmptyPQ() 
	{ 
		return (mPQ.size() == 0); 
	}

	double peekPQ() 
	{ 
		return mPQ.top_key(); 
	}

	Vertex popPQ(Graph &g) 
	{
		size_t top_index = mPQ.top_idx();
		Vertex top_vertex = mIndexToNode[top_index];
		mPQ.remove_min();
		return top_vertex;
	}

	void printPQ()
	{
		mPQ.print();
	}

	double calculateKey(Vertex &node) 
	{  
		return std::min(getDistance(node), getDistanceLookahead(node)); 
	}

	std::vector<Vertex> followBackpointers(double& costOut) 
	{
		costOut = getDistance(mGoalNode);
		if (costOut == INF_VAL)
			return std::vector<Vertex>();

		std::vector<Vertex> finalPath;
		finalPath.push_back(mGoalNode);
		Vertex curBack = mPrev[mGoalNode];

		// std::cout<<"Map contains:"<<std::endl;
		// for(auto i = mPrev.begin();i!=mPrev.end();i++)
			// std::cout<<i->first<<" : "<<i->second<<std::endl;
		while (curBack != mStartNode) {
			// std::cout<<"In follow back pointers loop! :("<<std::endl;
			finalPath.push_back(curBack);
			curBack = mPrev[curBack];
		}
		finalPath.push_back(mStartNode);

		std::reverse(finalPath.begin(), finalPath.end());
		return finalPath;
	}


	std::vector<Vertex> getGraphVertices(Graph &graph) 
	{
		std::vector<Vertex> graphVertices;

		std::pair<VertexIter, VertexIter> vp;
		for (vp = vertices(graph); vp.first != vp.second; ++vp.first) {
			Vertex curVertex = *vp.first;
			graphVertices.push_back(curVertex);
		}

		return graphVertices;
	}

	std::vector<Vertex> getPredecessors(Vertex &v, Graph &g)
	{
		// std::cout<<"In getPredecessors"<<std::endl;
		return getNeighbors(v,g);
	}

	std::vector<Vertex> getNeighbors(Vertex &v, Graph &g)
	{
		VPIndexMap index_map = get(&VProp::vertex_index, g);

		// std::cout<<"IN getNeighbors "<<std::endl;
		std::vector <Vertex> neighbors;
		neighbors = tpg.getNeighborsImplicitTPG(v, g, left_map, right_map, g[mGoalNode].state);
		// std::cout<<"Neighbours of "<<index_map[v]<<" are: ";
		for(auto i=neighbors.begin();i!=neighbors.end();++i)
		{
			// std::cout<<index_map[*i]<<" ";
			if(!mDistance.count(*i)) //doesn't exist in distance map
			{
				// std::cout<<"Not present in distance map!"<<std::endl;
				mDistance[*i]=INF_VAL;
				mDistanceLookahead[*i]=INF_VAL;
			}
			// else
				// std::cout<<"Present in distance map!"<<std::endl;
		}
		// std::cout<<" Out of getNeighbors"<<std::endl;
		return neighbors;

	}


public:
	LPAStarTPG() {}
	
	void initLPA(Graph &composite_map, Graph & _left_map, Graph & _right_map, Vertex &start, Vertex &goal) 
	{
		left_map = _left_map;
		right_map = _right_map;
		// display_graph(left_map);
		// display_graph(right_map);

		mPQ.reset();
		mDistance.clear();
		mDistanceLookahead.clear();
		mPrev.clear();
		mIndexToNode.clear();

		mStartNode = start;
		mGoalNode = goal;

		tpg.populateMaps(left_map,right_map,composite_map,mStartNode,mGoalNode);


		mDistanceLookahead[mStartNode] = 0.0;
		mDistance[mStartNode] = INF_VAL;

		mDistanceLookahead[mGoalNode] = INF_VAL;
		mDistance[mGoalNode] = INF_VAL;

		insertPQ(composite_map, mStartNode, 0.0);
	}

	void updateVertex(Graph &g, Vertex &u) 
	{
		EPWeightMap edgeLengthMap = get(&EProp::weight, g);
		VPIndexMap index_map = get(&VProp::vertex_index, g);

		// std::cout<<"Index of start node:"<<index_map[mStartNode]<<std::endl;

		// std::cout<<"In update vertex PQ for vertex "<<index_map[u]<<std::endl;

		// std::cout<<"Edge between "<<index_map[u]<<" and start has exists status: "<<edge(u, mStartNode, g).second<<std::endl;

		if (u != mStartNode) {
			std::vector<double> lookaheadValues;

			auto preds = getPredecessors(u, g);
			for (auto &curPred : preds) 
			{
				Edge curEdge = boost::edge(curPred, u, g).first;
				double edgeWeight = edgeLengthMap[curEdge];
				// std::cout<<"Vertex index: "<<index_map[curPred]<<" getDistance: "<<getDistance(curPred)<<" edgeWeight: "<<edgeWeight<<std::endl;
				// std::cout<<"Press to cont: ";
				// std::cin.get();
				double curLookahead = getDistance(curPred) + edgeWeight;
				lookaheadValues.push_back(curLookahead);
			}
			// std::cout<<"Normal exit";

			if (lookaheadValues.size() == 0) 
			{
				mDistanceLookahead[u] = INF_VAL;
			} 
			else 
			{
				std::vector<double>::iterator it = std::min_element(
						std::begin(lookaheadValues), std::end(lookaheadValues));

				double minLookahead = *it;
				mDistanceLookahead[u] = minLookahead;

				// std::cout<<"Current lookaheadValue:"<<minLookahead<<std::endl;
			}
		}
		// std::cout<<"BEFORE CONTAINS"<<std::endl;

		if (containsPQ(g, u))
			removePQ(g, u);

		// std::cout<<" Insert Status : "<<mDistance.count(u)<<std::endl;
		// std::cout<<" getDistance(u) "<<getDistance(u)<<" getDistanceLookahead(u): "<<getDistanceLookahead(u)<<std::endl;
		if (getDistance(u) != getDistanceLookahead(u))
		{
			// std::cout<<"Inserting into queue! "<<std::endl;
			insertPQ(g, u, calculateKey(u));
		}
	}

	bool updatePredecessor(Graph &g, Vertex &u, Vertex &v) 
	{
		// std::cout<<"IN updatePredecessor"<<std::endl;
		if (v == mStartNode)
			return false;

		bool u_relied_on = false;
		if (mPrev.count(v)) {
			Vertex v_pred = mPrev[v];
			u_relied_on = v_pred == u;
		}

		double v_look_old = getDistanceLookahead(v);

		EPWeightMap edgeLengthMap = get(&EProp::weight, g);
		Edge uv_edge = boost::edge(u, v, g).first;
		double uv_weight = edgeLengthMap[uv_edge];

		double v_look_u = getDistance(u) + uv_weight;

		if (u_relied_on) // u was previously relied upon
		{
			// if dist through u decreased, then u is still best; just update value
			if (v_look_u == v_look_old) {
				return false;
			} else if (v_look_u < v_look_old) {
				mDistanceLookahead[v] = v_look_u;
				return true;
			} else // dist through u increased
			{
				// so we need to search for a potentially new best predecessessor
				double v_look_best = INF_VAL;
				Vertex v_pred_best;

				std::vector<Vertex> v_preds = getPredecessors(v, g);
				// std::cout<<"In updatePredecessor, Press [ENTER] to continue!";
				// std::cin.get();
				for (auto curPred : v_preds) {
					Edge predEdge = boost::edge(curPred, v, g).first;
					double predEdgeWeight = edgeLengthMap[predEdge];
					double curLookahead = getDistance(curPred) + predEdgeWeight;

					if (curLookahead < v_look_best) {
						v_look_best = curLookahead;
						v_pred_best = curPred;
					}
					// std::cout<<"IN CUR PRED LOOP"<<std::endl;
				}
				// std::cout<<"In complete updatePredecessor, Press [ENTER] to continue!";
				// std::cin.get();

				if (v_look_best != INF_VAL)
					mPrev[v] = v_pred_best;

				if (v_look_best == v_look_old) {
					return false;
				} else {
					mDistanceLookahead[v] = v_look_best;
					return true;
				}
			}
		} else // some other (existing) predecessor was used by v
		{
			if (v_look_u < v_look_old) // dist through u is better
			{
				mPrev[v] = u;
				mDistanceLookahead[v] = v_look_u;
				return true;
			} else // u is not better
			{
				return false;
			}
		}
		// std::cout<<"Out of get predecessor"<<std::endl;
	}

	std::vector<Vertex> computeShortestPath( Graph &g, double& costOut) 
	{
		// std::cout<<"In computeShortestPath"<<std::endl;
		while (!isEmptyPQ() &&
					 (peekPQ() < calculateKey(mGoalNode) ||
						getDistanceLookahead(mGoalNode) != getDistance(mGoalNode))) {

			// std::cout<<"In while loop!"<<std::endl;

			// std::cout<<"Press ENter to continue: ";
			// std::cin.get();
			// double curCost = peekPQ();
			Vertex u = popPQ(g);


			// std::cout<<"PQ contains: ";
			// printPQ();
			// std::cout<<(g[u].vertex_index)<<std::endl;

			if (getDistance(u) > getDistanceLookahead(u)) {
				// Force it to be consistent.
				mDistance[u] = getDistanceLookahead(u);

				// std::cout<<"forced to be consistent"<<std::endl;

				std::vector<Vertex> neighbors = getNeighbors(u, g);
				for (auto successor : neighbors) {
					// std::cout<<"In succesors loop!"<<std::endl;
					if (updatePredecessor(g, u, successor))
						updateVertex(g, successor);
				}
			} else {
				mDistance[u] = INF_VAL;
				updateVertex(g, u);

				std::vector<Vertex> neighbors = getNeighbors(u, g);
				for (auto successor : neighbors) {
					if (updatePredecessor(g, u, successor))
						updateVertex(g, successor);
				}
			}
			// std::cout<<std::endl<<"Graph size: "<<boost::num_vertices(g)<<"vertices and "<<boost::num_edges(g)<<" edges.\n";
			// std::cout<<"PQ contains: ";
			// printPQ();
		}
		// std::cout<<"Out of computeShortestPath into followBackpointers"<<std::endl;
		// Actual path.
		return followBackpointers(costOut);
	}

}; // class LPAStar

#endif // LPA_STAR_

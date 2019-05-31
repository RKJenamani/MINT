#include "cartestianProductGenerator.h"
#include "PRLStackUtilCPG.h"
#include "LPAStar.h"
#include "priority_queue.h"

double INF_VAL = std::numeric_limits<double>::max();

namespace po = boost::program_options;

///////////LPAStar------

double LPAStar::getDistance(Vertex &v) { return mDistance[v]; }

double LPAStar::getDistanceLookahead(Vertex &v) {
  return mDistanceLookahead[v];
}

void LPAStar::updatePQ(Graph &g, Vertex &v, double newPriority) {
  VPIndexMap index_map = get(&VProp::vertex_index, g);
  mPQ.update(index_map[v], newPriority);
}

void LPAStar::insertPQ(Graph &g, Vertex &v, double priority) {
  VPIndexMap index_map = get(&VProp::vertex_index, g);
  size_t node_index = index_map[v];

  mPQ.insert(node_index, priority);

  // NOTE: Also associate index with node, since mPQ only stores indices.
  mIndexToNode[node_index] = v;
}

void LPAStar::removePQ(Graph &g, Vertex &v) {
  VPIndexMap index_map = get(&VProp::vertex_index, g);
  mPQ.remove(index_map[v]);
}

bool LPAStar::containsPQ(Graph &g, Vertex &v) {
  VPIndexMap index_map = get(&VProp::vertex_index, g);
  return mPQ.contains(index_map[v]);
}

bool LPAStar::isEmptyPQ() { return (mPQ.size() == 0); }

double LPAStar::peekPQ() { return mPQ.top_key(); }

Vertex LPAStar::popPQ(Graph &g) {
  size_t top_index = mPQ.top_idx();
  Vertex top_vertex = mIndexToNode[top_index];

  mPQ.remove_min();
  return top_vertex;
}

double LPAStar::calculateKey(Vertex &node) {
  return std::min(getDistance(node), getDistanceLookahead(node));
}

std::vector<Vertex> LPAStar::followBackpointers(double& costOut) {
  costOut = getDistance(mGoalNode);

  // Check if we actually reached the goal vertex. If we didn't, fail and
  // cry (by returning an empty vector).
  if (costOut == INF_VAL)
    return std::vector<Vertex>();

  std::vector<Vertex> finalPath;
  finalPath.push_back(mGoalNode);
  Vertex curBack = mPrev[mGoalNode];

  while (curBack != mStartNode) {
    finalPath.push_back(curBack);
    curBack = mPrev[curBack];
  }
  finalPath.push_back(mStartNode);

  std::reverse(finalPath.begin(), finalPath.end());
  return finalPath;
}

void LPAStar::initLPA(Graph &g, Vertex &start, Vertex &goal) {
  // NOTE: Reset LPA* data structures each time in case we're doing multiple
  // fitness evaluations.
  mPQ.reset();
  mDistance.clear();
  mDistanceLookahead.clear();
  mPrev.clear();
  mIndexToNode.clear();

  mStartNode = start;
  mGoalNode = goal;

  std::vector<Vertex> initNodes = getGraphVertices(g);
  for (auto &vertex : initNodes) {
    mDistance[vertex] = INF_VAL;
    mDistanceLookahead[vertex] = INF_VAL;
  }

  mDistanceLookahead[mStartNode] = 0.0;
  insertPQ(g, mStartNode, 0.0);
}

void LPAStar::updateVertex(Graph &g, Vertex &u) {
  EPWeightMap edgeLengthMap = get(&EProp::weight, g);

  if (u != mStartNode) {
    std::vector<double> lookaheadValues;

    auto preds = getPredecessors(u, g);
    for (auto &curPred : preds) {
      Edge curEdge = boost::edge(curPred, u, g).first;
      double edgeWeight = edgeLengthMap[curEdge];
      double curLookahead = getDistance(curPred) + edgeWeight;

      lookaheadValues.push_back(curLookahead);
    }

    if (lookaheadValues.size() == 0) {
      mDistanceLookahead[u] = INF_VAL;
    } else {
      std::vector<double>::iterator it = std::min_element(
          std::begin(lookaheadValues), std::end(lookaheadValues));

      double minLookahead = *it;
      mDistanceLookahead[u] = minLookahead;
    }
  }

  if (containsPQ(g, u))
    removePQ(g, u);

  if (getDistance(u) != getDistanceLookahead(u))
    insertPQ(g, u, calculateKey(u));
}

bool LPAStar::updatePredecessor(Graph &g, Vertex &u, Vertex &v) {
  // start vertex dist lookahead is always zero
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
      for (auto curPred : v_preds) {
        Edge predEdge = boost::edge(curPred, v, g).first;
        double predEdgeWeight = edgeLengthMap[predEdge];
        double curLookahead = getDistance(curPred) + predEdgeWeight;

        if (curLookahead < v_look_best) {
          v_look_best = curLookahead;
          v_pred_best = curPred;
        }
      }

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
}

std::vector<Vertex> LPAStar::computeShortestPath(
  Graph &g,
  double& costOut
) {
  while (!isEmptyPQ() &&
         (peekPQ() < calculateKey(mGoalNode) ||
          getDistanceLookahead(mGoalNode) != getDistance(mGoalNode))) {
    double curCost = peekPQ();
    Vertex u = popPQ(g);

    if (getDistance(u) > getDistanceLookahead(u)) {
      // Force it to be consistent.
      mDistance[u] = getDistanceLookahead(u);

      std::vector<Vertex> neighbors = getNeighbors(u, g);
      for (auto successor : neighbors) {
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
  }

  // Actual path.
  return followBackpointers(costOut);
}

//////////////LPAStar--------------

Vertex connect_to_graph(Graph &g, Eigen::VectorXd &config, int &k)
{
	VPIndexMap indexMap = get(&VProp::vertex_index, g);
	VPStateMap stateMap = get(&VProp::state, g);
	EPWeightMap weightMap = get(&EProp::weight,g);

	int i=0;
	VertexIter vi,viend;
	for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi) 
		i++;

	Vertex node;
	node = add_vertex(g);
	stateMap[node] = config;
	indexMap[node] = i;

	std::unordered_map<int,Vertex> nodeMap;

	nodeMap[i]=node;

	priorityQueue pq;

	for (boost::tie(vi, viend) = vertices(g); vi != viend; ++vi)
	{
		if(indexMap[*vi]!=i)
		{
			// std::cout<<"Inserting "<<indexMap[node]<<std::endl;
			pq.insert(indexMap[*vi],(stateMap[node]-stateMap[*vi]).norm(),0.0);
			nodeMap[indexMap[*vi]]=*vi;
		}
	}
	// std::cout<<std::endl<<"Priority Queue: ";
	// pq1.printPQ();
	std::cout<<"Nodes connected to config: ";
	for(int j=0;j<k;j++)
	{
		int index = pq.pop();
		std::cout<<index<<" ";
		Edge curEdge;
		curEdge = (add_edge(nodeMap[index], node, g)).first;
		weightMap[curEdge]=(stateMap[node]-stateMap[nodeMap[index]]).norm();
	}
	std::cout<<std::endl;

	return node;
}

int main(int argc, char* argv[])
{
	po::options_description desc("Select File and Dimensions");
	desc.add_options()
		("help", "produce help message")
		("dimensions,d", po::value<int>()->required(), "set number of Dimensions")
		("graph,g",po::value<std::string>()->required(), " path to graphml file")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc,argv,desc), vm);

	if(vm.count("help")) {
		std::cout<<desc<<std::endl;
		return 1;
	}

	std::string filepath=vm["graph"].as<std::string>();
	int dim = vm["dimensions"].as<int>();

	Graph input_map;

	create_vertices(input_map,get(&VProp::state,input_map),filepath,dim,get(&EProp::prior,input_map));
	create_edges(input_map,get(&EProp::weight,input_map));
	VertexIter vi, vend;
	int i=0;
	for (boost::tie(vi, vend) = vertices(input_map); vi != vend; ++vi,++i)
	{
		put(&VProp::vertex_index,input_map,*vi,i);
	}

	std::cout<<"GRAPH LOADED! "<<std::endl;    

	Graph right_map = input_map;

  	Graph left_map(i);

  	VertexIter ui, uend;
  	std::unordered_map<int,Vertex> morphNodeMap;

  	i=0;
  	for (boost::tie(ui, uend) = vertices(input_map), boost::tie(vi, vend) = vertices(left_map) ; vi != vend; ++vi,++ui,++i)
  	{
  		put(&VProp::vertex_index,left_map,*vi,i);

  		morphNodeMap[i]=*vi;

  		Eigen::VectorXd new_config(7);
  		Eigen::VectorXd temp_config;

  		temp_config = input_map[*ui].state;

  		// std::cout<<temp_config(6);

  		new_config << -temp_config(0), temp_config(1), -temp_config(2), temp_config(3), -temp_config(4), temp_config(5), -temp_config(6);
  		// new_config << temp_config(0), temp_config(1), temp_config(2), temp_config(3), temp_config(4), temp_config(5), temp_config(6);

  		put(&VProp::state,left_map,*vi,new_config);
  	}

  	VPIndexMap oldIndexMap = get(&VProp::vertex_index,input_map);

  	EPWeightMap newWeightMap = get(&EProp::weight,left_map);

  	EdgeIter ei, ei_end, fi, fiend;
	for(boost::tie(ei,ei_end)=edges(input_map);ei!=ei_end;++ei)
	{
		Vertex u=source(*ei,input_map);
		Vertex v=target(*ei,input_map);
		Vertex new_u = morphNodeMap[oldIndexMap[u]];
		Vertex new_v = morphNodeMap[oldIndexMap[v]];

		Edge curEdge;
		curEdge = (add_edge(new_u, new_v, left_map)).first;
		newWeightMap[curEdge]=(left_map[new_u].state-left_map[new_v].state).norm();
	}

  	std::cout<<" NEW GRAPH LOADED! "<<std::endl;



	Eigen::VectorXd start_config(dim+dim);
	start_config <<	3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00, 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;
	// start_config <<	-5.00, -5.00, -5.00, -5.00;

	Eigen::VectorXd goal_config(dim+dim);
	goal_config << -4.42475, -0.900109, 0.672778, 2.29866, 0.792, -1.25398, -0.823374, 4.42475, -0.900109, -0.672778, 2.29866, -0.792, -1.25398, 0.823374;
	// goal_config << 5.00, 5.00, 5.00, 5.00;

	Eigen::VectorXd left_start_config(dim);
	left_start_config << start_config.segment(0,dim);
	Eigen::VectorXd right_start_config(dim);
	right_start_config << start_config.segment(dim,dim);
	Eigen::VectorXd left_goal_config(dim);
	left_goal_config << goal_config.segment(0,dim);
	Eigen::VectorXd right_goal_config(dim);
	right_goal_config << goal_config.segment(dim,dim);


	int k=20;

	Vertex left_start,left_end;
	Vertex right_start, right_end;

	std::cout<<"Left Start config :"<<left_start_config<<std::endl;
	left_start = connect_to_graph(left_map , left_start_config, k);

	std::cout<<"Left Goal config :"<<left_goal_config<<std::endl;
	left_end = connect_to_graph(left_map ,left_goal_config, k);

	std::cout<<"Right Start config :"<<right_start_config<<std::endl;
	right_start = connect_to_graph(right_map , right_start_config, k);

	std::cout<<"Right Goal config :"<<right_goal_config<<std::endl;
	right_end = connect_to_graph(right_map ,right_goal_config, k);


	std::cout<<"Size: "<<(i+2)*(i+2)<<std::endl;
	Graph herb_map((i+2)*(i+2));

	CartesianPG cpg;
	cpg.explicitCPG(left_map,dim,right_map,dim,herb_map);
	display_graph(herb_map);

	Vertex start,end;
	start = cpg.configToNodeCPG(start_config);
	end = cpg.configToNodeCPG(goal_config);

	VPIndexMap indexMap = get(&VProp::vertex_index, herb_map);

	std::cout<<"Index: "<<indexMap[start]<<" "<<indexMap[end]<<std::endl;

	EPWeightMap weightMap = get(&EProp::weight,herb_map);
	VPStateMap stateMap = get(&VProp::state, herb_map);
	LPAStar a;

	PRLStackUtilCPG checker;

	double pathcost;
	std::vector<Vertex> path;

	a.initLPA(herb_map,start,end);
	int numSearches = 0;

	std::cout<<"Input [ENTER] to start search: ";
  	std::cin.get();

	while(true)
	{
		numSearches++;
		EPEvaluatedMap edgeEvalMap = get(&EProp::evaluated,herb_map);

		VPIndexMap index_map = get(&VProp::vertex_index, herb_map);

		std::cout<< "[INFO]: Search "<<numSearches <<std::endl;
		std::vector<Vertex> shortestPath = a.computeShortestPath(herb_map,pathcost);
		for(Vertex nodes: shortestPath )
			std::cout<<index_map[nodes]<<" ";
		if( shortestPath.size()==0)
		{
			std::cout << "" <<std::endl;
			std::cout<< "[INFO]: ALL COLL" <<std::endl;
			path=std::vector<Vertex>();
			break;
		}

		bool collisionFree = true;
		for( int i=0;i<shortestPath.size()-1;i++)
		{
			Vertex curU = shortestPath.at(i);
			Vertex curV = shortestPath.at(i+1);

			Edge curEdge = boost::edge(curU,curV,herb_map).first;

			bool curEval = edgeEvalMap[curEdge];

			// std::cout<<std::endl<<"Eval: "<<curEval;

			if(!curEval)
			{

				edgeEvalMap[curEdge] = true;
				// bool col= checker.getCollisionStatus(herb_map[curU].state ,herb_map[curV].state);
				bool col = checker.getCollisionStatus(stateMap[curU],stateMap[curV]);
				std::cout<<"Edge between: "<<index_map[curU]<<" and "<<index_map[curV]<<" Col: "<<col;
				if(col)
				{
					put(&EProp::weight,herb_map,curEdge,INF_VAL);
					if (a.updatePredecessor(herb_map, curU, curV))
            			a.updateVertex(herb_map, curV);

          // Update reverse direction.
          			if (a.updatePredecessor(herb_map, curV, curU))
            			a.updateVertex(herb_map, curU);

					collisionFree=false;
					break;
				}
			}
		}
		if(collisionFree)
		{
			std::cout<<"Input [ENTER] to display path: ";
  			std::cin.get();
			std::cout<<std::endl<<"Shortest Path is of size:"<<shortestPath.size()<< " nodes"<<std::endl;

			std::cout<<std::endl<<"Shortest Path is of length:"<<pathcost<<std::endl;
			
			std::cout<<"Path: ";
			for(Vertex nodes: shortestPath )
			{
				std::cout<<index_map[nodes]<<" ";
			}
			std::cout<<std::endl<<"Input [ENTER] to execute path: ";
  			std::cin.get();

  			std::vector<Eigen::VectorXd> configs;

			for(auto vi=shortestPath.begin(); vi!=shortestPath.end();vi++)
				configs.push_back(herb_map[*vi].state);

			checker.executePathCPG(configs);
			std::cout<<std::endl<<"Input [ENTER] to exit: ";
  			std::cin.get();
			break;
		}
	}






	return 0;
}
#include "kShortestPaths.h"
#include <chrono>

namespace po = boost::program_options;

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
		("graph,g",po::value<std::string>()->required(), " path to left graphml file")
		("degree,k", po::value<int>()->required(), "set approximate degree")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc,argv,desc), vm);

	if(vm.count("help")) {
		std::cout<<desc<<std::endl;
		return 1;
	}

	std::string filepath=vm["graph"].as<std::string>();
	int dim = vm["dimensions"].as<int>();
	int k = vm["degree"].as<int>();

	Graph right_map;

	create_vertices(right_map,get(&VProp::state,right_map),filepath,dim,get(&EProp::prior,right_map));
	create_edges(right_map,get(&EProp::weight,right_map));
	VertexIter vi, vend;

	int i=0;
	for (boost::tie(vi, vend) = vertices(right_map); vi != vend; ++vi,++i)
	{
		put(&VProp::vertex_index,right_map,*vi,i);
	}

	std::cout<<"RIGHT GRAPH LOADED! "<<std::endl;    

	Eigen::VectorXd start_config(dim);
	start_config <<	3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;
	// start_config <<	-5.00, -5.00, -5.00, -5.00;

	Eigen::VectorXd goal_config(dim);
	goal_config << 4.42475, -0.900109, -0.672778, 2.29866, -0.792, -1.25398, 0.823374;
	// goal_config << 5.00, 5.00, 5.00, 5.00;

	// int k=20;

	Vertex start, end;

	std::cout<<"Start config :"<<start_config<<std::endl;
	start = connect_to_graph(right_map , start_config, k);

	std::cout<<"Goal config :"<<goal_config<<std::endl;
	end = connect_to_graph(right_map ,goal_config, k);

	VPIndexMap indexMap = get(&VProp::vertex_index, right_map);

	std::cout<<"Index: "<<indexMap[start]<<" "<<indexMap[end]<<std::endl;

	kShortestPaths a(right_map,start,end,5);

	std::vector<std::vector<Vertex> > mPaths = a.returnShortestPaths();

	for (auto path : mPaths)
	{
		for (auto vertex : path)
		{
			std::cout << vertex << " ";
		}
		std::cout << std::endl;
	}

	std::unordered_map <Vertex,Vertex> mapping;

	Graph herb_map;

	VPIndexMap oldIndexMap = get(&VProp::vertex_index, right_map);
	VPStateMap oldStateMap = get(&VProp::state, right_map);
	EPWeightMap oldWeightMap = get(&EProp::weight, right_map);

	VPIndexMap newIndexMap = get(&VProp::vertex_index, herb_map);
	VPStateMap newStateMap = get(&VProp::state, herb_map);
	EPWeightMap newWeightMap = get(&EProp::weight, herb_map);

	size_t index=0;
	for (auto path : mPaths)
	{
		// add start of each graph
		Vertex old_node = path.at(0);
		if(mapping.count(old_node)==0)
		{
			Vertex new_node;
			new_node = add_vertex(herb_map);
			newStateMap[new_node] = oldStateMap[old_node];
			newIndexMap[new_node] = index++;
			mapping[old_node]=new_node;
		}

		//rest of the path
		for(int i=1; i<path.size();i++)
		{
			old_node = path.at(i);			
			if(mapping.count(old_node)==0)
			{
				Vertex new_node;
				new_node = add_vertex(herb_map);
				newStateMap[new_node] = oldStateMap[old_node];
				newIndexMap[new_node] = index++;
				mapping[old_node]=new_node;
			}

			bool edge_status = edge(mapping[old_node], mapping[path.at(i-1)], herb_map).second;
			// std::cout<<mapping[old_node]<<" "<<mapping[path.at(i-1)]<<"Edge Status: "<<edge_status<<std::endl;
			if(!edge_status)
			{
				Edge curEdge;
				curEdge = (add_edge(mapping[old_node], mapping[path.at(i-1)], herb_map)).first;
				newWeightMap[curEdge]= (newStateMap[mapping[old_node]]-newStateMap[mapping[path.at(i-1)]]).norm();
			}
		}
	}
	display_graph(herb_map);


	// std::vector<Vertex> shortestPath = a.dijkstra(right_map,start,end);

	// if(shortestPath.size()==0)
	// {
	// 	std::cout<<"ALL COL!"<<std::endl;
	// 	return 0;
	// }

	// std::cout<<std::endl<<"Shortest Path is of size:"<<shortestPath.size()<< " nodes"<<std::endl;

	// std::cout<<"Path: ";
	// for(Vertex nodes: shortestPath )
	// {
	// 	std::cout<<indexMap[nodes]<<" ";
	// }

	return 0;
}
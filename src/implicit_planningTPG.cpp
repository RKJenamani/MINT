#include "LPAStarTPG.h"
#include "PRLStackUtilTPG.h"
#include "priority_queue.h"
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
		("left_graph,l",po::value<std::string>()->required(), " path to left graphml file")
		("right_graph,r",po::value<std::string>()->required(), " path to right graphml file")
		("degree,k", po::value<int>()->required(), "set approximate degree")
	;

	po::variables_map vm;
	po::store(po::parse_command_line(argc,argv,desc), vm);

	if(vm.count("help")) {
		std::cout<<desc<<std::endl;
		return 1;
	}

	std::string left_filepath=vm["left_graph"].as<std::string>();
	std::string right_filepath=vm["right_graph"].as<std::string>();
	int dim = vm["dimensions"].as<int>();
	int k = vm["degree"].as<int>();

	Graph right_map;

	create_vertices(right_map,get(&VProp::state,right_map),right_filepath,dim,get(&EProp::prior,right_map));
	create_edges(right_map,get(&EProp::weight,right_map));
	VertexIter vi, vend;
	int i=0;
	for (boost::tie(vi, vend) = vertices(right_map); vi != vend; ++vi,++i)
	{
		put(&VProp::vertex_index,right_map,*vi,i);
	}

	std::cout<<"RIGHT GRAPH LOADED! "<<std::endl;    

	Graph left_map;

	create_vertices(left_map,get(&VProp::state,left_map),left_filepath,dim,get(&EProp::prior,left_map));
	create_edges(left_map,get(&EProp::weight,left_map));
	i=0;
	for (boost::tie(vi, vend) = vertices(left_map); vi != vend; ++vi,++i)
	{
		put(&VProp::vertex_index,left_map,*vi,i);
	}

	std::cout<<"LEFT GRAPH LOADED! "<<std::endl;    


	Eigen::VectorXd start_config(dim+dim);
	start_config <<	3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00, 3.14, -1.57, 0.00, 0.00, 0.00, 0.00, 0.00;
	// start_config <<	-5.00, -5.00, -5.00, -5.00;

	Eigen::VectorXd goal_config(dim+dim);
	goal_config << 0.541593, -2, -2.8, -0.9, -4.76, -1.6, -3, 4.42475, -0.900109, -0.672778, 2.29866, -0.792, -1.25398, 0.823374;
	// goal_config << 5.00, 5.00, 5.00, 5.00;

	Eigen::VectorXd left_start_config(dim);
	left_start_config << start_config.segment(0,dim);
	Eigen::VectorXd right_start_config(dim);
	right_start_config << start_config.segment(dim,dim);
	Eigen::VectorXd left_goal_config(dim);
	left_goal_config << goal_config.segment(0,dim);
	Eigen::VectorXd right_goal_config(dim);
	right_goal_config << goal_config.segment(dim,dim);


	// int k=20;

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


	std::cout<<"Size: "<<2<<std::endl;
	Graph herb_map;

	VPIndexMap indexMap = get(&VProp::vertex_index, herb_map);
	EPWeightMap weightMap = get(&EProp::weight,herb_map);
	VPStateMap stateMap = get(&VProp::state, herb_map);

	Vertex start;
	start = add_vertex(herb_map);
	stateMap[start] = start_config;
	indexMap[start] = 0;

	Vertex end;
	end = add_vertex(herb_map);
	stateMap[end] = goal_config;
	indexMap[end] = 1;




	std::cout<<"Index: "<<indexMap[start]<<" "<<indexMap[end]<<std::endl;
	
	LPAStarTPG a;

	PRLStackUtilTPG checker;

	double pathcost;
	std::vector<Vertex> path;

	a.initLPA(herb_map,left_map,right_map,start,end);
	int numSearches = 0;

	std::cout<<"Input [ENTER] to start search: ";
		std::cin.get();

	std::chrono::time_point<std::chrono::system_clock> start_time, end_time, start_col_time, end_col_time, start_compute_time, end_compute_time; 


	start_time = std::chrono::system_clock::now(); 

	std::chrono::duration<double> collision_elapsed_seconds;
	std::chrono::duration<double> computeShortestPath_elapsed_seconds;


	while(true)
	{
		numSearches++;
		EPEvaluatedMap edgeEvalMap = get(&EProp::evaluated,herb_map);

		VPIndexMap index_map = get(&VProp::vertex_index, herb_map);

		std::cout<< "[INFO]: Search "<<numSearches <<std::endl;

		start_compute_time = std::chrono::system_clock::now(); 

		std::vector<Vertex> shortestPath = a.computeShortestPath(herb_map,pathcost);
		std::cout<<std::endl<<"Graph size: "<<boost::num_vertices(herb_map)<<"vertices and "<<boost::num_edges(herb_map)<<" edges.\n";

		end_compute_time = std::chrono::system_clock::now();

		computeShortestPath_elapsed_seconds += (end_compute_time - start_compute_time) ; 
		// for(Vertex nodes: shortestPath )
		// 	std::cout<<index_map[nodes]<<" ";

		// std::cout<<std::endl<<"Path is of length:"<<pathcost<<std::endl;

		// for(Vertex nodes: shortestPath )
		// {
		// 	std::cout<<"State:";
		// 	for(int num =0; num<14;num++)
		// 		std::cout<<stateMap[nodes](num)<<" ";
		// 	std::cout<<std::endl;
		// }
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
				start_col_time = std::chrono::system_clock::now(); 
				bool col = checker.getCollisionStatus(stateMap[curU],stateMap[curV]);
				end_col_time = std::chrono::system_clock::now(); 

				collision_elapsed_seconds += (end_col_time - start_col_time); 

				// bool col =false;
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

			end_time = std::chrono::system_clock::now(); 

			std::chrono::duration<double> elapsed_seconds = end_time - start_time; 
            std::cout<< "Time taken for total search: " << elapsed_seconds.count() << "s\n"; 
            std::cout<< "Time taken for collision checking: "<<collision_elapsed_seconds.count()<<"s\n";
            std::cout<< "Time taken for computeShortestPath iterations: "<<computeShortestPath_elapsed_seconds.count()<<"s\n";

            size_t graph_size_vertices = boost::num_vertices(herb_map);
            size_t graph_size_edges = boost::num_edges(herb_map);
            std::cout<<"Final graph size: "<<graph_size_vertices<<"vertices and "<<graph_size_edges<<" edges.\n";

			std::cout<<"Input [ENTER] to display path: ";
				std::cin.get();
			std::cout<<std::endl<<"Shortest Path is of size:"<<shortestPath.size()<< " nodes"<<std::endl;

			std::cout<<std::endl<<"Shortest Path is of length:"<<pathcost<<std::endl;
			
			std::cout<<"Path: ";
			for(Vertex nodes: shortestPath )
			{
				std::cout<<index_map[nodes]<<" ";
			}

			// for(Vertex nodes: shortestPath )
			// {
			// 	std::cout<<"State:";
			// 	for(int num =0; num<14;num++)
			// 		std::cout<<stateMap[nodes](num)<<" ";
			// 	std::cout<<std::endl;
			// }

			std::cout<<std::endl<<"Input [ENTER] to execute path: ";
			std::cin.get();

			std::vector<Eigen::VectorXd> configs;

			for(auto vi=shortestPath.begin(); vi!=shortestPath.end();vi++)
				configs.push_back(herb_map[*vi].state);

			checker.executePathTPG(configs);
			std::cout<<std::endl<<"Input [ENTER] to exit: ";
				std::cin.get();
			break;
		}
	}

	return 0;
}
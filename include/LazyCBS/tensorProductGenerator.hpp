#ifndef TPG_GENERATOR_
#define TPG_GENERATOR_

#include<bits/stdc++.h>

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <cmath>
#include <stdlib.h>

#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <Eigen/Dense>

#include "BGLDefinitions.hpp"

#define PAUSE_PENALTY 0.1

namespace std //As  we are using map with Eigen::VectorXd as key!
{
	template<> struct less<Eigen::VectorXd>
	{
		bool operator() (Eigen::VectorXd const& a, Eigen::VectorXd const& b) const
		{
			assert(a.size()==b.size());
			for(size_t i=0;i<a.size();++i)
			{
				if(a[i]<b[i]) return true;
				if(a[i]>b[i]) return false;
			}
			return false;
		}
	};
}

namespace LazyCBS {

using namespace BGL_DEFINITIONS;

template<typename T>
struct matrix_hash : std::unary_function<T, size_t> {
  std::size_t operator()(T const& matrix) const {
    // Note that it is oblivious to the storage order of Eigen matrix (column- or
    // row-major). It will give you the same hash value for two different matrices if they
    // are the transpose of each other in different storage order.
    size_t seed = 0;
    for (size_t i = 0; i < matrix.size(); ++i) {
      auto elem = *(matrix.data() + i);
      seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};


class TensorPG
{
public: //Change to private once this is working
	// std::unordered_map<int,Vertex> indexToNodeMap;

	ompl::base::StateSpacePtr mSpace;

	std::unordered_map<int,bool> neighborsAddedMap;
	// std::unordered_map<Eigen::VectorXd, Vertex, matrix_hash<Eigen::VectorXd>> configToNodeMap;

	std::unordered_map<Eigen::VectorXd,Vertex, matrix_hash<Eigen::VectorXd>> configToNodeMap;

	std::unordered_map<Eigen::VectorXd,Vertex, matrix_hash<Eigen::VectorXd>> leftConfigToNodeMap; //used for implicit graph
	std::unordered_map<Eigen::VectorXd,Vertex, matrix_hash<Eigen::VectorXd>> rightConfigToNodeMap; //used for implicit map

public:

	TensorPG(ompl::base::StateSpacePtr _space)
	: mSpace(_space)
	{}

	void populateMaps(Graph &left_map, Graph &right_map, CompositeGraph &composite_map, CompositeVertex &start, CompositeVertex &goal)
	{
		VertexIter vi, viend;

		for(boost::tie(vi, viend)= vertices(left_map); vi!=viend;++vi)
			leftConfigToNodeMap[left_map[*vi].state]=*vi;

		for(boost::tie(vi, viend)= vertices(right_map); vi!=viend;++vi)
			rightConfigToNodeMap[right_map[*vi].state]=*vi;

		Eigen::VectorXd start_config(mSpace->getDimension());
		double* start_values = composite_map[start].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
		// start_config << start_values[0], start_values[1], start_values[2], start_values[3],
			// start_values[4], start_values[5], start_values[6], start_values[7], start_values[8],
			// start_values[9], start_values[10], start_values[11], start_values[12], start_values[13];
		for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
		{
			start_config[ui] = start_values[ui];
		}

		Eigen::VectorXd goal_config(mSpace->getDimension());
		double* goal_values = composite_map[goal].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
		// goal_config << goal_values[0], goal_values[1], goal_values[2], goal_values[3],
		// 	goal_values[4], goal_values[5], goal_values[6], goal_values[7], goal_values[8],
		// 	goal_values[9], goal_values[10], goal_values[11], goal_values[12], goal_values[13];
		for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
		{
			goal_config[ui] = goal_values[ui];
		}
		configToNodeMap[start_config]=start;
		configToNodeMap[goal_config]=goal;
	}
	

	void explicitTPG(Graph & g1, int dim1, Graph &g2, int dim2, CompositeGraph &new_map, 
		Eigen::VectorXd left_goal_config, Eigen::VectorXd right_goal_config)
	{
		// std::cout<<"In explicit: "<<std::endl;
		// std::map<Vector14d, Vertex> configToNodeMap;
		
		int i=0;
		// std::cout<<"Debug: ";
  // 		std::cin.get();

		VertexIter vi1, viend1;
		VertexIter vi2, viend2;

		for (boost::tie(vi1, viend1) = vertices(g1) ; vi1 != viend1; ++vi1)
		{
			for (boost::tie(vi2, viend2) = vertices(g2); vi2 != viend2; ++vi2, ++i)
			{
				CompositeVertex new_node;
				new_node = add_vertex(new_map);				
				new_map[new_node].vertex_index=i;
				Eigen::VectorXd new_config(dim1+dim2);
				new_config << g1[*vi1].state , g2[*vi2].state;
				configToNodeMap[new_config] = new_node;

				utils::StateWrapperPtr newState(new utils::StateWrapper(mSpace));
				ompl::base::State *ver_state{newState->state};
				double *values{ver_state->as<ompl::base::RealVectorStateSpace::StateType>()->values};
				for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
				{
					values[ui] = new_config[ui];
				}
				new_map[new_node].state = newState;
				new_map[new_node].heuristic = std::max( (g1[*vi1].state-left_goal_config).norm(),
					(g2[*vi2].state- right_goal_config).norm());
			}
		}

		EdgeIter ei1, eiend1;
		EdgeIter ei2, eiend2;

		size_t num_of_edges=0;
		for(boost::tie(ei1,eiend1)=edges(g1);ei1!=eiend1;++ei1)
		{
			// std::cout<<num_of_edges++<<std::endl;
			Vertex u_1=source(*ei1,g1);
			Vertex v_1=target(*ei1,g1);

			for(boost::tie(ei2,eiend2)=edges(g2);ei2!=eiend2;++ei2)
			{
				Vertex u_2=source(*ei2,g2);
				Vertex v_2=target(*ei2,g2);

				Eigen::VectorXd new_config_source(dim1+dim2);
				Eigen::VectorXd new_config_target(dim1+dim2);
				
				new_config_source << g1[u_1].state , g2[u_2].state;
				new_config_target << g1[v_1].state , g2[v_2].state;

				CompositeEdge curEdge;

				// std::cout<< configToNodeMap[new_config_source];
				curEdge = (boost::add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
				new_map[curEdge].length=std::max((new_config_source.segment(0,dim1)-new_config_target.segment(0,dim1)).norm(),
						(new_config_source.segment(dim1,dim2)-new_config_target.segment(dim1,dim2)).norm());

				new_config_source << g1[u_1].state , g2[v_2].state;
				new_config_target << g1[v_1].state , g2[u_2].state;

				// std::cout<< configToNodeMap[new_config_source];
				curEdge = (boost::add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
				new_map[curEdge].length=std::max((new_config_source.segment(0,dim1)-new_config_target.segment(0,dim1)).norm(),
						(new_config_source.segment(dim1,dim2)-new_config_target.segment(dim1,dim2)).norm());		
			}
		}

		/// adding edges of cartesian product graphs

		for(boost::tie(ei1,eiend1)=edges(g1);ei1!=eiend1;++ei1)
		{
			Vertex u=source(*ei1,g1);
			Vertex v=target(*ei1,g1);

			for (boost::tie(vi2, viend2) = vertices(g2); vi2 != viend2; ++vi2)
			{
				Eigen::VectorXd new_config_source(dim1+dim2);
				Eigen::VectorXd new_config_target(dim1+dim2);
				
				new_config_source << g1[u].state , g2[*vi2].state;
				new_config_target << g1[v].state , g2[*vi2].state;

				CompositeEdge curEdge;

				if(right_goal_config.isApprox(g2[*vi2].state))
				{
					// std::cout<< configToNodeMap[new_config_source];
					curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
					new_map[curEdge].length=(new_config_source-new_config_target).norm();			
				}
				else
				{
					// std::cout<< configToNodeMap[new_config_source];
					curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
					new_map[curEdge].length=(new_config_source-new_config_target).norm() + PAUSE_PENALTY ;	
				}
			}
		}
		for(boost::tie(ei2,eiend2)=edges(g2);ei2!=eiend2;++ei2)
		{
			Vertex u=source(*ei2,g2);
			Vertex v=target(*ei2,g2);

			for (boost::tie(vi1, viend1) = vertices(g1); vi1 != viend1; ++vi1)
			{
				Eigen::VectorXd new_config_source(dim1+dim2);
				Eigen::VectorXd new_config_target(dim1+dim2);
				
				new_config_source << g1[*vi1].state , g2[u].state;
				new_config_target << g1[*vi1].state , g2[v].state;

				CompositeEdge curEdge;

				if(left_goal_config.isApprox(g1[*vi1].state))
				{
					curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
					new_map[curEdge].length=(new_config_source-new_config_target).norm();	
				}
				else
				{
					curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
					new_map[curEdge].length=(new_config_source-new_config_target).norm()+ PAUSE_PENALTY;					
				}		
			}
		}
	}

	std::vector<CompositeVertex> getNeighborsImplicitTPG(CompositeVertex &node, CompositeGraph &composite_map, Graph &left_map, Graph &right_map, 
		Eigen::VectorXd goal_config)
	{
		// std::cout<<"NUMBER OF EDGES: "<<boost::num_edges(composite_map)<<std::endl;
		// std::cout<<"In getNeighborsImplicitCPG"<<std::endl;

		// std::cout<<"Press [ENTER] to continue";
		// std::cin.get();

		std::vector<CompositeVertex> neighbors;
		CompositeNeighborIter ai, aiend;
		NeighborIter ai_l, aiend_l, ai_r, aiend_r ;

		if(neighborsAddedMap.count(composite_map[node].vertex_index))
		{
			for (boost::tie(ai, aiend) = adjacent_vertices(node, composite_map); ai != aiend; ++ai) 
			{
				neighbors.push_back(*ai);
			}
			return neighbors;
		}
		// std::cout<<" "<<indexMap[node]<<" ";
		neighborsAddedMap[composite_map[node].vertex_index]=true;

		VertexIter vi, viend;

		double* composite_config_values = 
			composite_map[node].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

		Eigen::VectorXd composite_config(mSpace->getDimension());
		for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
		{
			composite_config[ui] = composite_config_values[ui];
		}
		
		int dim = (composite_config.size()/2);

		Eigen::VectorXd left_goal_config = goal_config.segment(0,dim);
		Eigen::VectorXd right_goal_config = goal_config.segment(dim,dim);


		Eigen::VectorXd left_config = composite_config.segment(0,dim);
		Eigen::VectorXd right_config = composite_config.segment(dim,dim);


		Vertex left_map_node=leftConfigToNodeMap[left_config]; //Create Map!!
		Vertex right_map_node=rightConfigToNodeMap[right_config]; //Create Map!!

		size_t size = num_vertices(composite_map);

		// std::cout<<"DEBUG:";
		// std::cout<<left_map[left_map_node].vertex_index<<std::endl;
		// std::cout<<left_map[left_map_node].state<<std::endl;

		// size_t count_left = 0;
		// for (boost::tie(ai_l, aiend_l) = adjacent_vertices(left_map_node, left_map); ai_l != aiend_l; ++ai_l) 
		// {
		// 	std::cout<<left_map[*ai_l].vertex_index<<" ";
		// 	count_left++;
		// }
		// std::cout<<"FINAL:"<<count_left<<std::endl;

		// std::cout<<"Middle of getNeighborsImplicitCPG"<<std::endl;

		//Adding Edges of tensor Map!!
		// std::cout<<"Press [ENTER] to continue";
		// std::cin.get();

		// int edges_count =0;
		// int config_vertices_count=0;
		// int add_vertices_count=0;
		int count_val=0;
		for (boost::tie(ai_l, aiend_l) = adjacent_vertices(left_map_node, left_map); ai_l != aiend_l; ++ai_l) 
		{
			for(boost::tie(ai_r,aiend_r)=adjacent_vertices(right_map_node,right_map);ai_r!=aiend_r; ++ai_r)
			{
				count_val++;
				// std::cout<<"LOOP! "<<std::endl;
				Eigen::VectorXd adjacent_left_config = left_map[*ai_l].state;
				Eigen::VectorXd adjacent_right_config = right_map[*ai_r].state;
				Eigen::VectorXd adjacent_composite_config(dim+dim);
				adjacent_composite_config << adjacent_left_config , adjacent_right_config ;

				// std::cout<<"Adjacent composite config state: "<<adjacent_composite_config<<std::endl;

				if(configToNodeMap.count(adjacent_composite_config))
				{
					// std::cout<<composite_map[configToNodeMap[adjacent_composite_config]].vertex_index<<" ";
					// std::cin.get();
					CompositeVertex new_node = configToNodeMap[adjacent_composite_config];
					// std::cout<<"Already Exists!! "<<indexMap[configToNodeMap[adjacent_composite_config]];
					bool edge_status = edge(new_node, node, composite_map).second;
					// std::cout<<"Edge Status: "<<edge_status<<std::endl;
					if(!edge_status)
					{
						std::pair<CompositeEdge,bool> curEdge = boost::add_edge(new_node, node, composite_map);
						// graph[newEdge.first].length = mSpace->distance(graph[new_node].state->state, graph[node].state->state);
						composite_map[curEdge.first].length = std::max( (adjacent_composite_config.segment(0,dim)-composite_config.segment(0,dim)).norm(),
							(adjacent_composite_config.segment(dim,dim)-composite_config.segment(dim,dim)).norm());
						composite_map[curEdge.first].prior = 1.0;

						// Edge curEdge;
						// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
						// curEdge = (add_edge(new_node, node, composite_map)).first;
						// weightMap[curEdge]=(stateMap[new_node].segment(0,dim)-stateMap[node].segment(0,dim)).norm()+
						// 	(stateMap[new_node].segment(dim,dim)-stateMap[node].segment(dim,dim)).norm();
						// weightMap[curEdge]=std::max((stateMap[new_node].segment(0,dim)-stateMap[node].segment(0,dim)).norm(),
							// (stateMap[new_node].segment(dim,dim)-stateMap[node].segment(dim,dim)).norm());
						// edges_count++;
					}
					neighbors.push_back(new_node);
					// config_vertices_count++;
					continue;
				}

				utils::StateWrapperPtr newState(new utils::StateWrapper(mSpace));
				ompl::base::State *ver_state{newState->state};
				double *values{ver_state->as<ompl::base::RealVectorStateSpace::StateType>()->values};
				for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
				{
					values[ui] = adjacent_composite_config[ui];
				}

				CompositeVertex new_node;
				new_node = add_vertex(composite_map);
				composite_map[new_node].state = newState;
				composite_map[new_node].vertex_index = boost::num_vertices(composite_map) - 1;
				composite_map[new_node].heuristic = std::max( (adjacent_composite_config.segment(0,dim)-goal_config.segment(0,dim)).norm(),
					(adjacent_composite_config.segment(dim,dim)-goal_config.segment(dim,dim)).norm());

				configToNodeMap[adjacent_composite_config]=new_node;
				// add_vertices_count++;

				// Edge curEdge;
				std::pair<CompositeEdge,bool> curEdge = boost::add_edge(new_node, node, composite_map);
				composite_map[curEdge.first].length = std::max( (adjacent_composite_config.segment(0,dim)-composite_config.segment(0,dim)).norm(),
					(adjacent_composite_config.segment(dim,dim)-composite_config.segment(dim,dim)).norm());
				composite_map[curEdge.first].prior = 1.0;
				// bool edge_status = edge(new_node, node, composite_map).second;
				// std::cout<<"Edge Status: "<<edge_status<<std::endl;
				// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
				// curEdge = (add_edge(new_node, node, composite_map)).first;
				// weightMap[curEdge]=(stateMap[new_node].segment(0,dim)-stateMap[node].segment(0,dim)).norm() + 
				// 	(stateMap[new_node].segment(dim,dim)-stateMap[node].segment(dim,dim)).norm();
				// weightMap[curEdge]=std::max((stateMap[new_node].segment(0,dim)-stateMap[node].segment(0,dim)).norm(),
					// (stateMap[new_node].segment(dim,dim)-stateMap[node].segment(dim,dim)).norm());
				neighbors.push_back(new_node);
				// edges_count++;
			}
		}
		// std::cout<<"Sim added:"<<count_val<<std::endl;
		count_val=0;

		// std::cout<<"Press [ENTER] to continue";
		// std::cin.get();

		//Adding Edges of Left Map!!
		for (boost::tie(ai_l, aiend_l) = adjacent_vertices(left_map_node, left_map); ai_l != aiend_l; ++ai_l) 
		{
			count_val++;
			// std::cout<<"LEFT! "<<std::endl;
			Vertex curNeighbor = *ai_l;
			Eigen::VectorXd adjacent_left_config = left_map[*ai_l].state;
			Eigen::VectorXd adjacent_composite_config(dim+dim);
			adjacent_composite_config << adjacent_left_config , right_config ;

			// std::cout<<"Adjacent composite config state: "<<adjacent_composite_config<<std::endl;

			if(configToNodeMap.count(adjacent_composite_config))
			{
				// std::cin.get();
				CompositeVertex new_node = configToNodeMap[adjacent_composite_config];
				// std::cout<<"Already Exists!! "<<indexMap[configToNodeMap[adjacent_composite_config]];
				bool edge_status = edge(new_node, node, composite_map).second;
				// std::cout<<"Edge Status: "<<edge_status<<std::endl;
				if(!edge_status)
				{
					std::pair<CompositeEdge,bool> curEdge = boost::add_edge(new_node, node, composite_map);
					composite_map[curEdge.first].prior = 1.0;
					// Edge curEdge;
					// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
					// curEdge = (add_edge(new_node, node, composite_map)).first;

					if(right_goal_config.isApprox(composite_config.segment(dim,dim)))
					{
						composite_map[curEdge.first].length=(adjacent_composite_config- composite_config).norm();
					}
					else
					{
						composite_map[curEdge.first].length=(adjacent_composite_config- composite_config).norm() + PAUSE_PENALTY;
						// weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();
					}
				}
				neighbors.push_back(new_node);
				// edges_count++;
				// config_vertices_count++;
				continue;
			}

			utils::StateWrapperPtr newState(new utils::StateWrapper(mSpace));
			ompl::base::State *ver_state{newState->state};
			double *values{ver_state->as<ompl::base::RealVectorStateSpace::StateType>()->values};
			for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
			{
				values[ui] = adjacent_composite_config[ui];
			}

			CompositeVertex new_node;
			new_node = add_vertex(composite_map);
			composite_map[new_node].state = newState;
			composite_map[new_node].vertex_index = boost::num_vertices(composite_map) - 1;
			composite_map[new_node].heuristic = std::max( (adjacent_composite_config.segment(0,dim)-goal_config.segment(0,dim)).norm(),
				(adjacent_composite_config.segment(dim,dim)-goal_config.segment(dim,dim)).norm());
			configToNodeMap[adjacent_composite_config]=new_node;

			std::pair<CompositeEdge,bool> curEdge = boost::add_edge(new_node, node, composite_map);
			composite_map[curEdge.first].prior = 1.0;

			// Edge curEdge;
			// bool edge_status = edge(new_node, node, composite_map).second;
			// std::cout<<"Edge Status: "<<edge_status<<std::endl;
			// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
			// curEdge = (add_edge(new_node, node, composite_map)).first;

			if(right_goal_config.isApprox(composite_config.segment(dim,dim)))
			{
				composite_map[curEdge.first].length=(adjacent_composite_config- composite_config).norm();
			}
			else
			{
				composite_map[curEdge.first].length=(adjacent_composite_config- composite_config).norm()+PAUSE_PENALTY;
				// weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();
			}
			// edges_count++;
			// add_vertices_count++;
			neighbors.push_back(new_node);
		}

		// std::cout<<"Left added:"<<count_val<<std::endl;
		count_val =0;
		// std::cout<<"Press [ENTER] to continue";
		// std::cin.get();

		// std::cout<<"Middle of getNeighborsImplicitCPG"<<std::endl;
		// std::cin.get();
		//Adding Edges of right Map!
		for (boost::tie(ai_r, aiend_r) = adjacent_vertices(right_map_node, right_map); ai_r != aiend_r; ++ai_r) 
		{
			count_val++;
			// std::cout<<"RIGHT!!"<<std::endl;
			// std::cout<<"Size: "<<size<<std::endl;
			Vertex curNeighbor = *ai_r;
			Eigen::VectorXd adjacent_right_config = right_map[*ai_r].state;
			Eigen::VectorXd adjacent_composite_config(dim+dim);
			adjacent_composite_config << left_config, adjacent_right_config;

			if(configToNodeMap.count(adjacent_composite_config))
			{
				CompositeVertex new_node = configToNodeMap[adjacent_composite_config];
				// std::cout<<"Already Exists!! "<<indexMap[configToNodeMap[adjacent_composite_config]];
				bool edge_status = edge(new_node, node, composite_map).second;
				// std::cout<<"Edge Status: "<<edge_status<<std::endl;
				if(!edge_status)
				{
					std::pair<CompositeEdge,bool> curEdge = boost::add_edge(new_node, node, composite_map);
					composite_map[curEdge.first].prior = 1.0;
					// Edge curEdge;
					// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
					// curEdge = (add_edge(new_node, node, composite_map)).first;

					if(left_goal_config.isApprox(composite_config.segment(0,dim)))
					{
						composite_map[curEdge.first].length=(adjacent_composite_config- composite_config).norm();
					}
					else
					{
						composite_map[curEdge.first].length=(adjacent_composite_config- composite_config).norm() + PAUSE_PENALTY;
						// weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();
					}
				}

				// edges_count++;
				// config_vertices_count++;
				neighbors.push_back(new_node);
				continue;
			}

			utils::StateWrapperPtr newState(new utils::StateWrapper(mSpace));
			ompl::base::State *ver_state{newState->state};
			double *values{ver_state->as<ompl::base::RealVectorStateSpace::StateType>()->values};
			for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
			{
				values[ui] = adjacent_composite_config[ui];
			}

			CompositeVertex new_node;
			new_node = add_vertex(composite_map);
			composite_map[new_node].state = newState;
			composite_map[new_node].vertex_index = boost::num_vertices(composite_map) - 1;
			composite_map[new_node].heuristic = std::max( (adjacent_composite_config.segment(0,dim)-goal_config.segment(0,dim)).norm(),
				(adjacent_composite_config.segment(dim,dim)-goal_config.segment(dim,dim)).norm());
			configToNodeMap[adjacent_composite_config]=new_node;

			std::pair<CompositeEdge,bool> curEdge = boost::add_edge(new_node, node, composite_map);
			composite_map[curEdge.first].prior = 1.0;

			if(left_goal_config.isApprox(composite_config.segment(0,dim)))
			{
				composite_map[curEdge.first].length=(adjacent_composite_config- composite_config).norm();
			}
			else
			{
				composite_map[curEdge.first].length=(adjacent_composite_config- composite_config).norm() + PAUSE_PENALTY;
				// weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();
			}
			// edges_count++;
			// add_vertices_count++;
			neighbors.push_back(new_node);
		}
		// std::cout<<"Right added:"<<count_val<<std::endl;
		// std::cout<<"Out of getNeighborsImplicitCPG"<<std::endl;

		// std::cout<<"Size: "<<size<<std::endl;
		// std::cout<<"EDGES COUNT: "<<edges_count<<std::endl;
		// std::cout<<"add_vertices_count: "<<add_vertices_count<<std::endl;
		// std::cout<<"config_vertices_count: "<<config_vertices_count<<std::endl;
		// std::cout<<"Press [ENTER] to continue";
		// std::cin.get();

		return neighbors;

	}
	// Vertex indexToNodeTPG(int index)
	// {
	// 	return indexToNodeMap[index];
	// }
	Vertex configToNodeTPG(Eigen::VectorXd config)
	{
		return configToNodeMap[config];
	}
	Vertex leftConfigToNodeTPG(Eigen::VectorXd config)
	{
		return leftConfigToNodeMap[config];
	}
	Vertex rightConfigToNodeTPG(Eigen::VectorXd config)
	{
		return rightConfigToNodeMap[config];
	}
};

}  // namespace LazyCBS

#endif
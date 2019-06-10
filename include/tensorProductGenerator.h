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

#include "BGLDefinitions.h"

#define PAUSE_PENALTY 1 

using namespace BGL_DEFINITIONS;


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
	std::unordered_map<int,Vertex> indexToNodeMap;

	std::unordered_map<int,bool> neighborsAddedMap;
	// std::unordered_map<Eigen::VectorXd, Vertex, matrix_hash<Eigen::VectorXd>> configToNodeMap;

	std::unordered_map<Eigen::VectorXd,Vertex, matrix_hash<Eigen::VectorXd>> configToNodeMap;

	std::unordered_map<Eigen::VectorXd,Vertex, matrix_hash<Eigen::VectorXd>> leftConfigToNodeMap; //used for implicit graph
	std::unordered_map<Eigen::VectorXd,Vertex, matrix_hash<Eigen::VectorXd>> rightConfigToNodeMap; //used for implicit map

public:

	TensorPG(){}

	void populateMaps(Graph &left_map, Graph &right_map, Graph & composite_map, Vertex &start, Vertex &goal)
	{
		VertexIter vi, viend;

		for(boost::tie(vi, viend)= vertices(left_map); vi!=viend;++vi)
			leftConfigToNodeMap[left_map[*vi].state]=*vi;

		for(boost::tie(vi, viend)= vertices(right_map); vi!=viend;++vi)
			rightConfigToNodeMap[right_map[*vi].state]=*vi;

		VPStateMap stateMap = get(&VProp::state,composite_map);

		configToNodeMap[stateMap[start]]=start;
		configToNodeMap[stateMap[goal]]=goal;
	}
	

	void explicitTPG(Graph & g1, int dim1, Graph &g2, int dim2, Graph &new_map, 
		Eigen::VectorXd left_goal_config, Eigen::VectorXd right_goal_config)
	{
		std::cout<<"In explicit: "<<std::endl;
		// std::map<Vector14d, Vertex> configToNodeMap;
		
		int i=0;
		// std::cout<<"Debug: ";
  // 		std::cin.get();

		VertexIter vi1, viend1;
		VertexIter vi2, viend2;
		VertexIter vi_new, viend_new;

		for (boost::tie(vi1, viend1) = vertices(g1) ,boost::tie(vi_new, viend_new)= vertices(new_map) ; vi1 != viend1; ++vi1)
		{
			for (boost::tie(vi2, viend2) = vertices(g2); vi2 != viend2; ++vi2, ++vi_new, ++i)
			{
				put(&VProp::vertex_index,new_map,*vi_new,i);
				
				indexToNodeMap[i]=*vi_new;

				Eigen::VectorXd new_config(dim1+dim2);

				new_config << g1[*vi1].state , g2[*vi2].state;

				configToNodeMap[new_config] = *vi_new;
				// std::cout<<"New config: "<<new_config<<std::endl;
				put(&VProp::state,new_map,*vi_new,new_config);
			}
		}

		EPWeightMap newWeightMap = get(&EProp::weight,new_map);

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

				Edge curEdge;

				// std::cout<< configToNodeMap[new_config_source];
				curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
				newWeightMap[curEdge]=(new_config_source.segment(0,dim1)-new_config_target.segment(0,dim1)).norm()+(new_config_source.segment(dim1,dim2)-new_config_target.segment(dim1,dim2)).norm();

				new_config_source << g1[u_1].state , g2[v_2].state;
				new_config_target << g1[v_1].state , g2[u_2].state;

				// std::cout<< configToNodeMap[new_config_source];
				curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
				newWeightMap[curEdge]=(new_config_source.segment(0,dim1)-new_config_target.segment(0,dim1)).norm()+(new_config_source.segment(dim1,dim2)-new_config_target.segment(dim1,dim2)).norm();			
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

				Edge curEdge;

				if(left_goal_config==new_config_source.segment(0,dim1) || left_goal_config==new_config_target.segment(0,dim1)
					|| right_goal_config==new_config_source.segment(dim1,dim2) || right_goal_config==new_config_target.segment(dim1,dim2))
				{
					// std::cout<< configToNodeMap[new_config_source];
					curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
					newWeightMap[curEdge]=(new_config_source-new_config_target).norm();			
				}
				else
				{
					// std::cout<< configToNodeMap[new_config_source];
					curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
					newWeightMap[curEdge]=(new_config_source-new_config_target).norm() + PAUSE_PENALTY ;	
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

				Edge curEdge;

				if(left_goal_config==new_config_source.segment(0,dim1) || left_goal_config==new_config_target.segment(0,dim1)
					|| right_goal_config==new_config_source.segment(dim1,dim2) || right_goal_config==new_config_target.segment(dim1,dim2))
				{
					curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
					newWeightMap[curEdge]=(new_config_source-new_config_target).norm();	
				}
				else
				{
					curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
					newWeightMap[curEdge]=(new_config_source-new_config_target).norm()+ PAUSE_PENALTY;					
				}		
			}
		}
	}

	std::vector<Vertex> getNeighborsImplicitTPG(Vertex &node, Graph &composite_map, Graph &left_map, Graph &right_map, 
		Eigen::VectorXd goal_config)
	{
		// std::cout<<"In getNeighborsImplicitCPG"<<std::endl;

		// std::cout<<"In get getNeighborsImplicitCPG";
		// std::cin.get();
		VPIndexMap indexMap = get(&VProp::vertex_index,composite_map);


		std::vector<Vertex> neighbors;
		NeighborIter ai, aiend, ai_l, aiend_l, ai_r, aiend_r ;

		if(neighborsAddedMap.count(indexMap[node]))
		{
			for (boost::tie(ai, aiend) = adjacent_vertices(node, composite_map); ai != aiend; ++ai) 
			{
				neighbors.push_back(*ai);
			}
			return neighbors;
		}
		// std::cout<<" "<<indexMap[node]<<" ";
		neighborsAddedMap[indexMap[node]]=true;

		VPStateMap stateMap = get(&VProp::state, composite_map);
		EPWeightMap weightMap = get(&EProp::weight, composite_map);

		VertexIter vi, viend;

		Eigen::VectorXd composite_config = stateMap[node];
		int dim = (composite_config.size()/2);

		Eigen::VectorXd left_goal_config = goal_config.segment(0,dim);
		Eigen::VectorXd right_goal_config = goal_config.segment(dim,dim);


		Eigen::VectorXd left_config = composite_config.segment(0,dim);
		Eigen::VectorXd right_config = composite_config.segment(dim,dim);


		Vertex left_map_node=leftConfigToNodeMap[left_config]; //Create Map!!
		Vertex right_map_node=rightConfigToNodeMap[right_config]; //Create Map!!

		size_t size = num_vertices(composite_map);

		// std::cout<<"Middle of getNeighborsImplicitCPG"<<std::endl;

		//Adding Edges of tensor Map!!
		for (boost::tie(ai_l, aiend_l) = adjacent_vertices(left_map_node, left_map); ai_l != aiend_l; ++ai_l) 
		{
			for(boost::tie(ai_r,aiend_r)=adjacent_vertices(right_map_node,right_map);ai_r!=aiend_r; ++ai_r)
			{
				// std::cout<<"LOOP! "<<std::endl;
				Eigen::VectorXd adjacent_left_config = left_map[*ai_l].state;
				Eigen::VectorXd adjacent_right_config = right_map[*ai_r].state;
				Eigen::VectorXd adjacent_composite_config(dim+dim);
				adjacent_composite_config << adjacent_left_config , adjacent_right_config ;

				// std::cout<<"Adjacent composite config state: "<<adjacent_composite_config<<std::endl;

				if(configToNodeMap.count(adjacent_composite_config))
				{
					// std::cin.get();
					Vertex new_node = configToNodeMap[adjacent_composite_config];
					// std::cout<<"Already Exists!! "<<indexMap[configToNodeMap[adjacent_composite_config]];
					bool edge_status = edge(new_node, node, composite_map).second;
					// std::cout<<"Edge Status: "<<edge_status<<std::endl;
					if(!edge_status)
					{

						Edge curEdge;
						// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
						curEdge = (add_edge(new_node, node, composite_map)).first;
						weightMap[curEdge]=(stateMap[new_node].segment(0,dim)-stateMap[node].segment(0,dim)).norm()+(stateMap[new_node].segment(dim,dim)-stateMap[node].segment(dim,dim)).norm();
					}
					neighbors.push_back(new_node);
					continue;
				}

				Vertex new_node;
				new_node = add_vertex(composite_map);
				stateMap[new_node] = adjacent_composite_config;
				indexMap[new_node] = size++;
				configToNodeMap[adjacent_composite_config]=new_node;

				Edge curEdge;
				// bool edge_status = edge(new_node, node, composite_map).second;
				// std::cout<<"Edge Status: "<<edge_status<<std::endl;
				// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
				curEdge = (add_edge(new_node, node, composite_map)).first;
				weightMap[curEdge]=(stateMap[new_node].segment(0,dim)-stateMap[node].segment(0,dim)).norm()+(stateMap[new_node].segment(dim,dim)-stateMap[node].segment(dim,dim)).norm();
				neighbors.push_back(new_node);
			}
		}


		//Adding Edges of Left Map!!
		for (boost::tie(ai, aiend) = adjacent_vertices(left_map_node, left_map); ai != aiend; ++ai) 
		{
			// std::cout<<"LEFT! "<<std::endl;
			Vertex curNeighbor = *ai;
			Eigen::VectorXd adjacent_left_config = left_map[*ai].state;
			Eigen::VectorXd adjacent_composite_config(dim+dim);
			adjacent_composite_config << adjacent_left_config , right_config ;

			// std::cout<<"Adjacent composite config state: "<<adjacent_composite_config<<std::endl;

			if(configToNodeMap.count(adjacent_composite_config))
			{
				// std::cin.get();
				Vertex new_node = configToNodeMap[adjacent_composite_config];
				// std::cout<<"Already Exists!! "<<indexMap[configToNodeMap[adjacent_composite_config]];
				bool edge_status = edge(new_node, node, composite_map).second;
				// std::cout<<"Edge Status: "<<edge_status<<std::endl;
				if(!edge_status)
				{
					Edge curEdge;
					// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
					curEdge = (add_edge(new_node, node, composite_map)).first;

					if(left_goal_config==stateMap[new_node].segment(0,dim) || left_goal_config==stateMap[node].segment(0,dim)
					|| right_goal_config==stateMap[node].segment(dim,dim))
					{
						weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();
					}
					else
					{
						weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm() + PAUSE_PENALTY;
					}
				}
				neighbors.push_back(new_node);
				continue;
			}

			Vertex new_node;
			new_node = add_vertex(composite_map);
			stateMap[new_node] = adjacent_composite_config;
			indexMap[new_node] = size++;
			configToNodeMap[adjacent_composite_config]=new_node;

			Edge curEdge;
			// bool edge_status = edge(new_node, node, composite_map).second;
			// std::cout<<"Edge Status: "<<edge_status<<std::endl;
			// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
			curEdge = (add_edge(new_node, node, composite_map)).first;

			if(left_goal_config==stateMap[new_node].segment(0,dim) || left_goal_config==stateMap[node].segment(0,dim)
					|| right_goal_config==stateMap[node].segment(dim,dim))
			{
				weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();
			}
			else
			{
				weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm()+PAUSE_PENALTY;
			}
			neighbors.push_back(new_node);
		}

		// std::cout<<"Middle of getNeighborsImplicitCPG"<<std::endl;
		// std::cin.get();
		//Adding Edges of right Map!
		for (boost::tie(ai, aiend) = adjacent_vertices(right_map_node, right_map); ai != aiend; ++ai) 
		{
			// std::cout<<"RIGHT!!"<<std::endl;
			// std::cout<<"Size: "<<size<<std::endl;
			Vertex curNeighbor = *ai;
			Eigen::VectorXd adjacent_right_config = right_map[*ai].state;
			Eigen::VectorXd adjacent_composite_config(dim+dim);
			adjacent_composite_config << left_config, adjacent_right_config;

			if(configToNodeMap.count(adjacent_composite_config))
			{
				Vertex new_node = configToNodeMap[adjacent_composite_config];
				// std::cout<<"Already Exists!! "<<indexMap[configToNodeMap[adjacent_composite_config]];
				bool edge_status = edge(new_node, node, composite_map).second;
				// std::cout<<"Edge Status: "<<edge_status<<std::endl;
				if(!edge_status)
				{
					Edge curEdge;
					// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
					curEdge = (add_edge(new_node, node, composite_map)).first;

					if(left_goal_config==stateMap[node].segment(0,dim) || right_goal_config==stateMap[node].segment(dim,dim)
					|| right_goal_config==stateMap[new_node].segment(dim,dim))
					{
						weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();
					}
					else
					{
						weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm() + PAUSE_PENALTY;
					}
				}

				
				neighbors.push_back(new_node);
				continue;
			}

			Vertex new_node;
			new_node = add_vertex(composite_map);
			stateMap[new_node] = adjacent_composite_config;
			indexMap[new_node] = size++;

			configToNodeMap[adjacent_composite_config]=new_node;

			Edge curEdge;
			curEdge = (add_edge(new_node, node, composite_map)).first;

			if(left_goal_config==stateMap[node].segment(0,dim) || right_goal_config==stateMap[node].segment(dim,dim)
			|| right_goal_config==stateMap[new_node].segment(dim,dim))
			{
				weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();
			}
			else
			{
				weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm() + PAUSE_PENALTY;
			}
			neighbors.push_back(new_node);
		}
		// std::cout<<"Out of getNeighborsImplicitCPG"<<std::endl;

		// std::cout<<"Size: "<<size<<std::endl;

		return neighbors;

	}
	Vertex indexToNodeTPG(int index)
	{
		return indexToNodeMap[index];
	}
	Vertex configToNodeTPG(Eigen::VectorXd config)
	{
		return configToNodeMap[config];
	}
};

#endif
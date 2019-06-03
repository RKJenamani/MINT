#ifndef CPG_GENERATOR_
#define CPG_GENERATOR_

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

class CartesianPG
{
public: //Change to private once this is working
	std::unordered_map<int,Vertex> indexToNodeMap;

	std::map<Eigen::VectorXd,Vertex> configToNodeMap;

	std::map<Eigen::VectorXd,Vertex> leftConfigToNodeMap; //used for implicit graph
	std::map<Eigen::VectorXd,Vertex> rightConfigToNodeMap; //used for implicit map

public:
	CartesianPG(){}

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

	void explicitCPG(Graph & g1, int dim1, Graph &g2, int dim2, Graph &new_map)
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
		// std::cout<<"Input 11[ENTER] to start search: ";
  // 		std::cin.get();

		VPIndexMap oldIndexMap1 = get(&VProp::vertex_index,g1);
		VPIndexMap oldIndexMap2 = get(&VProp::vertex_index,g2);

		EPWeightMap newWeightMap = get(&EProp::weight,new_map);

		EdgeIter ei1, eiend1;
		EdgeIter ei2, eiend2;
		EdgeIter ei_new, eiend_new;

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

				// std::cout<< configToNodeMap[new_config_source];
				curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
				newWeightMap[curEdge]=(new_config_source-new_config_target).norm();			
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
				curEdge = (add_edge(configToNodeMap[new_config_source], configToNodeMap[new_config_target], new_map)).first;
				newWeightMap[curEdge]=(new_config_source-new_config_target).norm();			
			}
		}
	}
	std::vector<Vertex> getNeighborsImplicitCPG(Vertex &node, Graph &composite_map, Graph &left_map, Graph &right_map) 
		//returns neighbors of node of composite_map using left_map and right_map and adds new nodes to composite_map
		//LPAStar map values to be set outside, called from LPAStar get Neighbours
	{
		// std::cout<<"In getNeighborsImplicitCPG"<<std::endl;

		VPIndexMap indexMap = get(&VProp::vertex_index,composite_map);
		VPStateMap stateMap = get(&VProp::state, composite_map);
		EPWeightMap weightMap = get(&EProp::weight, composite_map);

		VertexIter vi, viend;
		// std::cout<<"//////////////////////////////"<<std::endl;
		// for(boost::tie(vi,viend) = vertices(composite_map);vi!=viend; ++vi)
		// 		std::cout<<"Initial Composite Map nodes Iteration: "<<stateMap[*vi]<<std::endl;
		// std::cout<<"//////////////////////////////"<<std::endl;
		Eigen::VectorXd composite_config = stateMap[node];
		int dim = (composite_config.size()/2);
		Eigen::VectorXd left_config = composite_config.segment(0,dim);
		Eigen::VectorXd right_config = composite_config.segment(dim,dim);

		std::vector<Vertex> neighbors;

		Vertex left_map_node=leftConfigToNodeMap[left_config]; //Create Map!!
		Vertex right_map_node=rightConfigToNodeMap[right_config]; //Create Map!!

		size_t size = num_vertices(composite_map);

		NeighborIter ai;
		NeighborIter aiEnd;

		// std::cout<<"Middle of getNeighborsImplicitCPG"<<std::endl;

		//Adding Edges of Left Map!!
		for (boost::tie(ai, aiEnd) = adjacent_vertices(left_map_node, left_map); ai != aiEnd; ++ai) 
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

				// std::cout<<"Already Exists!! "<<indexMap[configToNodeMap[adjacent_composite_config]];
				neighbors.push_back(configToNodeMap[adjacent_composite_config]);
				continue;
			}

			Vertex new_node;
			new_node = add_vertex(composite_map);
			stateMap[new_node] = adjacent_composite_config;
			indexMap[new_node] = size++;
			configToNodeMap[adjacent_composite_config]=new_node;

			Edge curEdge;
			bool edge_status = edge(new_node, node, composite_map).second;
			// std::cout<<"Edge Status: "<<edge_status<<std::endl;
			// std::cout<<"Adding edge between "<<indexMap[new_node]<<" and "<<indexMap[node]<<std::endl;
			curEdge = (add_edge(new_node, node, composite_map)).first;
			weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();	
		




			for(boost::tie(vi,viend) = vertices(composite_map);vi!=viend; ++vi)
			{	
				// std::cout<<"Composite Map nodes Iteration: "<<stateMap[*vi]<<std::endl;
				if(indexMap[(*vi)]!=indexMap[new_node] && indexMap[(*vi)]!=indexMap[node])
				{
					Eigen::VectorXd left_V(dim);
					Eigen::VectorXd right_V(dim);

					left_V << stateMap[*vi].segment(0,dim);
					right_V << stateMap[*vi].segment(dim,dim);
					// std::cout<<"Iteration: ";
					// std::cout<<"IF to continue: ";
					// std::cin.get();

					// std::cout<<"Left_V: "<<left_V<<std::endl;

					if(left_V.isApprox(adjacent_left_config))
					{
						// std::cout<<"Left continue: ";
						// std::cin.get();
						Vertex right_vertex_U = rightConfigToNodeMap[right_V];
						Vertex right_vertex_V = rightConfigToNodeMap[right_config];

						bool edge_exists = edge(right_vertex_U, right_vertex_V, right_map).second;
						
						if(edge_exists)
						{
							Edge curEdge;
							curEdge = (add_edge(*vi, configToNodeMap[adjacent_composite_config], composite_map)).first;
							weightMap[curEdge]=(stateMap[*vi]-adjacent_composite_config).norm();			
						}					left_V << stateMap[*vi].segment(0,dim);

					}
					else if (right_V.isApprox(right_config))
					{
			// 			//check if edge exists between left_V and adjacent_left_config in left_map
			// 			//add edge
						// std::cout<<"Right to continue: ";
						// std::cout<<leftConfigToNodeMap[left_V]<<" "<<leftConfigToNodeMap[adjacent_left_config]<<std::endl;
						// std::cin.get();
						Vertex left_vertex_U = leftConfigToNodeMap[left_V];
						Vertex left_vertex_V = leftConfigToNodeMap[adjacent_left_config];
						bool edge_exists = edge(left_vertex_U, left_vertex_V, left_map).second;

						if(edge_exists)
						{

							Edge curEdge;
							bool edge_stat = edge(*vi, configToNodeMap[adjacent_composite_config], composite_map).second;
							// std::cout<<"Edge Status: "<<edge_stat<<std::endl;
							// std::cout<<composite_map[*vi].vertex_index<<" "<<configToNodeMap[adjacent_composite_config]<<std::endl;
							curEdge = (add_edge(*vi, configToNodeMap[adjacent_composite_config], composite_map)).first;
							weightMap[curEdge]=(stateMap[*vi]-adjacent_composite_config).norm();

	
						}		

					}
					// std::cout<<"Iteration: ";
					// std::cout<<"CLOSE to continue: ";
				}
			}

			neighbors.push_back(new_node);
		}

		// std::cout<<"Middle of getNeighborsImplicitCPG"<<std::endl;
		// std::cin.get();



		//Adding Edges of right Map!
		for (boost::tie(ai, aiEnd) = adjacent_vertices(right_map_node, right_map); ai != aiEnd; ++ai) 
		{
			// std::cout<<"RIGHT!!"<<std::endl;
			// std::cout<<"Size: "<<size<<std::endl;
			Vertex curNeighbor = *ai;
			Eigen::VectorXd adjacent_right_config = right_map[*ai].state;
			Eigen::VectorXd adjacent_composite_config(dim+dim);
			adjacent_composite_config << left_config, adjacent_right_config;

			if(configToNodeMap.count(adjacent_composite_config))
			{
				// std::cout<<"Already Exists!! "<<indexMap[configToNodeMap[adjacent_composite_config]];
				neighbors.push_back(configToNodeMap[adjacent_composite_config]);
				continue;
			}

			Vertex new_node;
			new_node = add_vertex(composite_map);
			stateMap[new_node] = adjacent_composite_config;
			indexMap[new_node] = size++;

			configToNodeMap[adjacent_composite_config]=new_node;

			Edge curEdge;
			curEdge = (add_edge(new_node, node, composite_map)).first;
			weightMap[curEdge]=(stateMap[new_node]-stateMap[node]).norm();

			for(boost::tie(vi,viend) = vertices(composite_map);vi!=viend; ++vi)
			{
				if(*vi!=new_node)
				{
					Eigen::VectorXd left_V(dim);
					Eigen::VectorXd right_V(dim);

					left_V << stateMap[*vi].segment(0,dim);
					right_V << stateMap[*vi].segment(dim,dim);

					if(right_V.isApprox(adjacent_right_config))
					{
						//check if edge exists between left_V and left_config in left_map
						//add edge
						Vertex left_vertex_U = leftConfigToNodeMap[left_V];
						Vertex left_vertex_V = leftConfigToNodeMap[left_config];
						bool edge_exists = edge(left_vertex_U, left_vertex_V, left_map).second;
						
						if(edge_exists)
						{
							Edge curEdge;
							curEdge = (add_edge(*vi, configToNodeMap[adjacent_composite_config], composite_map)).first;
							weightMap[curEdge]=(stateMap[*vi]-adjacent_composite_config).norm();			
						}
					}
					else if (left_V.isApprox(left_config))
					{
						//check if edge exists between right_V and adjacent_right_config in right_map
						//add edge
						Vertex right_vertex_U = rightConfigToNodeMap[right_V];
						Vertex right_vertex_V = rightConfigToNodeMap[adjacent_right_config];
						bool edge_exists = edge(right_vertex_U, right_vertex_V, right_map).second;
						
						if(edge_exists)
						{
							Edge curEdge;
							curEdge = (add_edge(*vi, configToNodeMap[adjacent_composite_config], composite_map)).first;
							weightMap[curEdge]=(stateMap[*vi]-adjacent_composite_config).norm();			
						}
					}
				}
			}
			neighbors.push_back(new_node);
		}
		// std::cout<<"Out of getNeighborsImplicitCPG"<<std::endl;

		// std::cout<<"Size: "<<size<<std::endl;
		return neighbors;

	}
	Vertex indexToNodeCPG(int index)
	{
		return indexToNodeMap[index];
	}
	Vertex configToNodeCPG(Eigen::VectorXd config)
	{
		return configToNodeMap[config];
	}
};

#endif
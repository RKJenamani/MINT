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
		bool operator() (Eigen::VectorXd const& a, Eigen::VectorXd const& b)
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
public:
	std::unordered_map<int,Vertex> indexToNodeMap;

	std::map<Eigen::VectorXd,Vertex> configToNodeMap;
public:
	CartesianPG(){}

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
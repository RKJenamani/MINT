#ifndef CBS_DEFINITIONS_
#define CBS_DEFINITIONS_
#include <bits/stdc++.h>
#include "BGLDefinitions.hpp"

namespace MINT {

using namespace BGL_DEFINITIONS;

struct Constraint
{
	Edge e;	//edge
	size_t t; //time
	Constraint(Edge _e, size_t _t) : e(_e), t(_t) {}
};

struct Element
{
	double cost;
	std::vector< Constraint > left_constraints;
	std::vector< Constraint > right_constraints;
	std::vector<Vertex> left_shortestPath;
	std::vector<Vertex> right_shortestPath;

	Element(double _cost, std::vector<Constraint> _left_constraints, std::vector<Constraint> _right_constraints,
		std::vector<Vertex> _left_shortestPath, std::vector<Vertex> _right_shortestPath): 
		cost(_cost), left_constraints(_left_constraints), right_constraints(_right_constraints),
		left_shortestPath(_left_shortestPath), right_shortestPath(_right_shortestPath) 
		{}

	inline bool operator < (const Element &b) const 
	{
    	if(cost<b.cost)
			return true;
		else
			return false;
	}
};

class CBSPriorityQueue
{
private:
	std::vector <Element> PQ;

	void min_heapify(int x)
	{
		int l=2*x;
		int r=2*x+1;
		int smallest = x;
		if(l<=(PQ.size()-1) && PQ[l]<PQ[x])
			smallest = l;
		if(r<=(PQ.size()-1) && PQ[r]<PQ[smallest])
			smallest = r;
		if(smallest!=x)
		{
			std::swap(PQ[smallest],PQ[x]);
			min_heapify(smallest);
		}
	}

public:
	CBSPriorityQueue()
	{ 
		Element a(-1,std::vector<Constraint>(),std::vector<Constraint>(),std::vector<Vertex>(),std::vector<Vertex>());
		PQ.push_back(a);
	}
	void reset()
	{
		PQ.clear();
		Element a(-1,std::vector<Constraint>(),std::vector<Constraint>(),std::vector<Vertex>(),std::vector<Vertex>());
		PQ.push_back(a);
	}
	int PQsize()
	{
		return PQ.size()-1;
	}
	double topKey()
	{
		return PQ[1].cost;
	}
	Element pop()
	{
		Element temp=PQ[1];
		PQ[1]=PQ[PQ.size()-1];
		PQ.erase(PQ.end()-1);
		min_heapify(1);
		return temp;
	}
	void insert(double _cost, std::vector<Constraint>  _left_constraints, std::vector<Constraint> _right_constraints,
		std::vector<Vertex> _left_shortestPath, std::vector<Vertex> _right_shortestPath)
	{
		// std::cout<<"Inserting : "<<v<<std::endl;
		Element a(_cost,_left_constraints,_right_constraints,_left_shortestPath,_right_shortestPath);
		PQ.push_back(a);
		// printPQ();
		int i=PQ.size()-1;
		while((i/2)>0)
		{
			if(PQ[i/2]<PQ[i])
				break;
			else
			{
				std::swap(PQ[i/2],PQ[i]);
				i=i/2;
			}
		}

	}
	void printPQ()
	{
		std::cout<<"Elements: "<<std::endl;
		for(int i=1;i<PQ.size();i++)
		{
			std::cout<<"Cost: "<<PQ[i].cost<<std::endl;
			std::cout<<"Left Constraints: "<<std::endl;
			//print constraints
			for(auto it=PQ[i].left_constraints.begin(); it != PQ[i].left_constraints.end(); it++)
			{
				std::cout<<"Edge: "<<(*it).e<<" Time: "<<(*it).t<<std::endl;
			}

			std::cout<<"Right Constraints: "<<std::endl;
			for(auto it=PQ[i].right_constraints.begin(); it != PQ[i].right_constraints.end(); it++)
			{
				std::cout<<"Edge: "<<(*it).e<<" Time: "<<(*it).t<<std::endl;
			}
			std::cout<<"shortestPaths: "<<std::endl;
			// not print paths as indexmap needed
			std::cout<<std::endl;
		}
		std::cout<<std::endl;
	}
};

} // namespace MINT

#endif
#ifndef K_SP_
#define K_SP_

#include <unordered_map>
#include <vector>

#include "priority_queue.h"
#include "BGLDefinitions.h"
#include "LoadGraphfromFile.h"
#include "visitors.h"
#include <bits/stdc++.h>

double INF_VAL = std::numeric_limits<double>::max();

using namespace BGL_DEFINITIONS;

class kShortestPaths
{
public:
	Graph g;
	    // Planner helpers
    Vertex mStartVertex;
    Vertex mGoalVertex;
    std::vector<std::vector<Vertex > > mPaths;

    // Planner parameters
    double mLookahead;

	kShortestPaths(Graph _g, Vertex _start, Vertex _goal, double _lookahead): 
		g(_g), mLookahead(_lookahead), mStartVertex(_start), mGoalVertex(_goal)
	{ }

	Edge getEdge(Vertex u, Vertex v)
	{
		Edge e;
		bool edgeExists;
		boost::tie(e, edgeExists) = edge(u, v, g);

		return e;
	}

    double pathLength(std::vector<Vertex> path)
	{
		double pathLength = 0;
		for(auto iterV = path.begin(); iterV != path.end() - 1; ++iterV)
		{
			Vertex u = *iterV;
			Vertex v = *(iterV + 1);
			Edge e = getEdge(u,v);
    		pathLength += g[e].weight;
		}
		return pathLength;
	}
	
	template<class TP>
	void addCandidatePaths(std::vector<Vertex> prevPath, TP &qCandidatePaths)
	{
		// http://courses.cs.vt.edu/cs6824/2014-spring/lectures/lecture-09-k-shortest-paths.pdf
		// std::map<Edge, double> lengths;

		Graph graph;
		copy_graph(g, graph);

		Edge uv;
		bool edge_exists;

		for (size_t i = 0; i < prevPath.size() - 1; ++i)
		{
			Vertex u = prevPath[i];

			// Hide incoming edges to u
			if (i > 0)
			{
				boost::tie(uv, edge_exists) = edge(prevPath[i-1], prevPath[i], graph);
				if (!edge_exists)
					throw std::runtime_error("Edge in previously found path does not exist!");
				graph[uv].weight = std::numeric_limits<double>::infinity();
			}

			// Hide the edge between p_j[i] and p_j[i+1] if p_j[:i+1] == prevPath[:i+1]
			for (std::size_t j = 0; j < mPaths.size(); ++j)
			{
				const auto &path = mPaths[j];
				
				bool prefixMatch = true;
				for (std::size_t n = 0; n <= i; ++n)
				{
					if (path[n] != prevPath[n])
					{
						prefixMatch = false;
						break;
					}
				}

				if (prefixMatch)
				{
					// Save original edge length
					if (i + 1 < path.size())
					{
						boost::tie(uv, edge_exists) = edge(path[i], path[i+1], graph);
						if (!edge_exists)
							throw std::runtime_error("Edge to previously found path does not exist!");

					graph[uv].weight = std::numeric_limits<double>::infinity();
					}
				}
			}

			std::vector<Vertex> candidate;
			candidate.insert(candidate.end(), prevPath.begin(), prevPath.begin()+i);

			// spur (path from prevPath[i] = u to goal in new graph)
			auto spur = dijkstra(graph, u, mGoalVertex);
			
			if (spur.empty())
				continue;
			
			candidate.insert(candidate.end(), spur.begin(), spur.end());

			qCandidatePaths.emplace(candidate);
		}
	}

	std::vector <Vertex> dijkstra(Graph graph, Vertex start, Vertex goal)
	{
		std::unordered_map<Vertex, Vertex> pred;
		std::unordered_map<Vertex, double> dist;

		try
		{
			dijkstra_shortest_paths(
			graph,
			start,
			boost::make_assoc_property_map(pred), // predecessor map
			boost::make_assoc_property_map(dist), // distance map
			get(&EProp::weight, graph), // weight map
			get(&VProp::vertex_index, graph), // index map
			std::less<double>(), // compare function
			boost::closed_plus<double>(std::numeric_limits<double>::infinity()), // combine function
			std::numeric_limits<double>::infinity(), // inf
			double(), // zero
			throw_when_visited<Graph, Vertex, Edge>(goal));
		}
		catch (const throw_visited_exception &e) {}

		// Construct the solution by starting at the goal and following predecessors
		// back to the start
		std::vector<Vertex> path;
		Vertex v = goal;
		while (v != start)
		{
			// no solution found
			if (v == pred[v])
			  break;

			path.push_back(v);
			v = pred[v];
		}
		
		// No solution found
		
		if (path.empty())
			return path;

		path.push_back(start);

		std::reverse(path.begin(), path.end());
		return path;
	}

	template<class TP>
	void computeKShortestPaths(TP &qCandidatePaths)
	{
		// Clear containers
		mPaths.clear();
		mPaths.reserve(mLookahead);
		
		qCandidatePaths.clear();

		std::vector<Vertex> path = dijkstra(g, mStartVertex, mGoalVertex);
		
		if(!path.empty())
		{
			qCandidatePaths.emplace(path);
			// The second part of this condition is essential for valid paths < K
			
			while (mPaths.size() < mLookahead && !qCandidatePaths.empty())
			{
				// Pop path with minimal path length
				std::vector<Vertex> path = *qCandidatePaths.begin();
				qCandidatePaths.erase(qCandidatePaths.begin());

				// Account for evaluated edges
				
				if (pathLength(path) == std::numeric_limits<double>::infinity())
					continue;

				// Avoid duplicates
				bool duplicate = false;
				for (const auto& prevPath : mPaths)
				{
					if (std::equal(path.begin(), path.end(), prevPath.begin()))
					{
						duplicate = true;
						break;
					}
				}

				if (duplicate)
					continue;

				mPaths.push_back(path);
				// std::cout<<"Path Added!"<<std::endl;

				// If enough paths are added, break
				if (mPaths.size() == mLookahead)
					break;

				// Add candidate paths from this path
				addCandidatePaths(path, qCandidatePaths);

				if (qCandidatePaths.empty())
					break;
			}

			// for (auto path : mPaths)
			// {
			// 	for (auto vertex : path)
			// 	{
			// 		std::cout << vertex << " ";
			// 	}
			// 	std::cout << std::endl;
			// }
		}
	}

	std::vector<std::vector<Vertex>> returnShortestPaths()
	{
		  // TODO (avk): Super convoluted, please simplify OMG. Use a different datastructure?
		auto cmpLValue = [&](std::vector<Vertex> left, std::vector<Vertex> right)
		{
			double lengthLeft = pathLength(left);
			double lengthRight = pathLength(right);

			if (lengthLeft < lengthRight)
				return true;
			
			if (lengthLeft > lengthRight)
				return false;
		
			if (lengthLeft == lengthRight)
			{
				if (left.size() < right.size())
					return true;
				if (left.size() > right.size())
					return false;
				if (left.size() == right.size())
				{
					for (int i = 0; i < left.size(); ++i)
					{
						if (left[i] < right[i])
							return true;
						if (left[i] > right[i])
							return false;
					}
				}
			}
			return false;
		};
		// Candidate Paths Queue: Used for Yen's Algorithm
		std::set<std::vector<Vertex>, decltype(cmpLValue)> qCandidatePaths(cmpLValue);

		std::cout<<"Press [Enter] to start search:";
		std::cin.get();
	    computeKShortestPaths(qCandidatePaths);
	    return mPaths;
	}
};

#endif
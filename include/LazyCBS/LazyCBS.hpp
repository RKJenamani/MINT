/* Authors: Rajat Kumar Jenamani */

#ifndef LazyCBS_HPP_
#define LazyCBS_HPP_

// STL headers
#include <vector>
#include <string> 
#include <unordered_set>
#include <queue>
#include <exception>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <algorithm>        // std::reverse
#include <cmath>            // pow, sqrt
#include <set>              // std::set
#include <assert.h>         // debug
#include <fstream>          // log
#include <chrono>           // time


// OMPL headers
#include <ompl/base/Planner.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

// LazyCBS defined headers
#include "util.hpp"
#include "BGLDefinitions.hpp"
#include "LoadGraphfromFile.hpp"
#include "CBSDefinitions.hpp"
#include "tensorProductGenerator.hpp"
#include "time_priority_queue.hpp"

std::string selector("composite"); //global selector for Collision Checking (options: composite, left and right)

// namespace std //As  we are using map with Eigen::VectorXd as key!
// {
// 	template<> struct less<Eigen::VectorXd>
// 	{
// 		bool operator() (Eigen::VectorXd const& a, Eigen::VectorXd const& b) const
// 		{
// 			assert(a.size()==b.size());
// 			for(size_t i=0;i<a.size();++i)
// 			{
// 				if(a[i]<b[i]) return true;
// 				if(a[i]>b[i]) return false;
// 			}
// 			return false;
// 		}
// 	};
// }
namespace LazyCBS {

using namespace BGL_DEFINITIONS;

void displayGraph(CompositeGraph &graph)
{
  // Obtain the image matrix
  cv::Mat image = cv::imread("/home/rajat/personalrobotics/ompl_ws/src/LazyCBS/data/obstacles/circle2D.png", 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

	CompositeEdgeIter ei, ei_end;
	for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei)
	{
		if(graph[*ei].isEvaluated==true)
		{
			if(graph[*ei].status == CollisionStatus::FREE)
			{
				double* u = graph[source(*ei, graph)].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  
				double* v = graph[target(*ei, graph)].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  

	    		cv::Point left_uPoint((int)(u[0]*numberOfColumns), (int)((1 - u[1])*numberOfRows));
			    cv::Point left_vPoint((int)(v[0]*numberOfColumns), (int)((1 - v[1])*numberOfRows));

			    cv::line(image, left_uPoint, left_vPoint, cv::Scalar(255, 0, 0), 3);
			  
			    cv::Point right_uPoint((int)(u[2]*numberOfColumns), (int)((1 - u[3])*numberOfRows));
			    cv::Point right_vPoint((int)(v[2]*numberOfColumns), (int)((1 - v[3])*numberOfRows));

			    cv::line(image, right_uPoint, right_vPoint, cv::Scalar(255, 0, 0), 3);
			}
			else
			{
				double* u = graph[source(*ei, graph)].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  
				double* v = graph[target(*ei, graph)].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  

	    		cv::Point left_uPoint((int)(u[0]*numberOfColumns), (int)((1 - u[1])*numberOfRows));
			    cv::Point left_vPoint((int)(v[0]*numberOfColumns), (int)((1 - v[1])*numberOfRows));

			    cv::line(image, left_uPoint, left_vPoint, cv::Scalar(0, 0, 255), 3);
			  
			    cv::Point right_uPoint((int)(u[2]*numberOfColumns), (int)((1 - u[3])*numberOfRows));
			    cv::Point right_vPoint((int)(v[2]*numberOfColumns), (int)((1 - v[3])*numberOfRows));

			    cv::line(image, right_uPoint, right_vPoint, cv::Scalar(0, 0, 255), 3);
			}
		}
	}
  cv::imshow("Solution Path", image);
  cv::waitKey(0);
}
void displayPath(CompositeGraph &graph, std::vector<CompositeVertex> shortestPath)
{
	// Obtain the image matrix
	cv::Mat image = cv::imread("/home/rajat/personalrobotics/ompl_ws/src/LazyCBS/data/obstacles/circle2D.png", 1);
	int numberOfRows = image.rows;
	int numberOfColumns = image.cols;

	for(size_t i=0; i<shortestPath.size()-1; i++)
	{
		double* u = graph[shortestPath.at(i)].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  
		double* v = graph[shortestPath.at(i+1)].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  

		cv::Point left_uPoint((int)(u[0]*numberOfColumns), (int)((1 - u[1])*numberOfRows));
		cv::Point left_vPoint((int)(v[0]*numberOfColumns), (int)((1 - v[1])*numberOfRows));

		cv::line(image, left_uPoint, left_vPoint, cv::Scalar(255, 0, 0), 3);

		cv::Point right_uPoint((int)(u[2]*numberOfColumns), (int)((1 - u[3])*numberOfRows));
		cv::Point right_vPoint((int)(v[2]*numberOfColumns), (int)((1 - v[3])*numberOfRows));

		cv::line(image, right_uPoint, right_vPoint, cv::Scalar(255, 0, 0), 3);
	}
  cv::imshow("Solution Path", image);
  cv::waitKey(0);
}


// template<typename T>
// struct matrix_hash : std::unary_function<T, size_t> {
// 	std::size_t operator()(T const& matrix) const {
// 		// Note that it is oblivious to the storage order of Eigen matrix (column- or
// 		// row-major). It will give you the same hash value for two different matrices if they
// 		// are the transpose of each other in different storage order.
// 		size_t seed = 0;
// 		for (size_t i = 0; i < matrix.size(); ++i) {
// 			auto elem = *(matrix.data() + i);
// 			seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
// 		}
// 		return seed;
// 	}
// };


struct pair_hash {
    template <class T1, class T2>
    std::size_t operator () (const std::pair<T1,T2> &p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        // Mainly for demonstration purposes, i.e. works but is overly simple
        // In the real world, use sth. like boost.hash_combine
        return h1 ^ h2;  
    }
};

/// The OMPL Planner class that implements the algorithm
class LazyCBS: public ompl::base::Planner
{
public:
	/// Constructor
	/// \param[in] si The OMPL space information manager
	explicit LazyCBS(const ompl::base::SpaceInformationPtr &si);

	/// \param[in] si The OMPL space information manager
	/// \param[in] roadmapFileName The path to the .graphml file that encodes the roadmap.
	/// \param[in] greediness The greediness to evaluate lazy shortest paths. Default is 1.
	LazyCBS(const ompl::base::SpaceInformationPtr &si,
		const std::string& leftRoadmapFileName, const std::string& rightRoadmapFileName);

	/// Destructor
	~LazyCBS(void);

	///////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////

	// /// Unordered set of graph vertices to track the lazy band.
	struct HashFunction
	{
		std::size_t operator()(const CompositeVertex& v) const
		{
			return v;
		}
	};
	typedef std::unordered_set<CompositeVertex, HashFunction> unorderedSet;

	///////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////

	// Setters and Getters

	/// Set left roadmap information.
	void setLeftRoadmapFileName(const std::string& leftRoadmapFileName);

	/// Set roadmap information.
	void setRightRoadmapFileName(const std::string& rightRoadmapFileName);

	/// Set connection radius
	void setConnectionRadius(double connectionRadius);

	/// Set collision checking radius
	void setCheckRadius(double checkRadius);

	/// Get roadmap information.
	std::string getLeftRoadmapFileName() const;

	/// Get roadmap information.
	std::string getRightRoadmapFileName() const;

	/// Get connection radius used to generate the graph.
	double getConnectionRadius() const;

	/// Get resolution of collision checking.
	double getCheckRadius() const;

	/// Get ID of start vertex.
	CompositeVertex getStartVertex() const;

	/// Get ID of goal vertex.
	CompositeVertex getGoalVertex() const;

	/// Get the shortest path cost.
	double getBestPathCost() const;

	///////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////

	// Internal evaluation methods
	/// Number of edges evaluated thus far.
	std::size_t getNumEdgeEvaluations() const;

	/// Number of edges rewired thus far.
	std::size_t getNumEdgeRewires() const;

	/// Time for edge evaluations.
	double getEdgeEvaluationsTime() const;

	/// Time for search.
	double getSearchTime() const;

	///////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////

	// OMPL required methods
	/// Set the problem definition and define the start, goal.
	void setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef);

	/// Solve the planning problem.
	ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);

	/// Setup the planner.
	void setup() override;

	/// Clear the planner setup.
	void clear() override;

private:

	// Planner parameters
	/// The pointer to the OMPL state space.
	const ompl::base::StateSpacePtr mSpace;

	/// The fixed left_graph denoting the left agent
	Graph left_graph;

	/// The fixed right_graph denoting the right agent
	Graph right_graph;

	/// The fixed roadmap over which the search is done.
	CompositeGraph graph;

	//// Used to populate the CompositeGraph
	TensorPG mTPG;

	/// Roadmap
	// boost::shared_ptr<utils::RoadmapFromFile<CompositeGraph, CompositeVPStateMap, utils::StateWrapper, CompositeEPLengthMap, CompositeEPPriorMap>> roadmapPtr;

	/// Path to the left roadmap.
	std::string mLeftRoadmapFileName;

	/// Path to the right roadmap.
	std::string mRightRoadmapFileName;

	/// Connection Radius [Strategy].
	double mConnectionRadius;

	/// Resolution to evaluate edges.
	double mCheckRadius;

	///////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////

	// Internal evaluation variables
	/// Number of edges evaluated.
	std::size_t mNumEdgeEvals{0u};

	/// Time for edge evaluations
	double mEdgeEvaluationsTime;

	/// Number of edges rewired.
	std::size_t mNumEdgeRewires{0u};

	/// Time for search
	double mSearchTime;

	/// Time for logging
	double mLogTime{0};

	/// Cost of optimal path.
	double mBestPathCost{std::numeric_limits<double>::infinity()};

	/// Track iterations
	std::size_t mIteration{0u};


	///////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////

	// Planner helpers
	/// Source vertex.
	CompositeVertex mStartVertex;

	/// Goal vertex.
	CompositeVertex mGoalVertex;

	Vertex mLeftStartVertex;
	Vertex mLeftGoalVertex;

	Vertex mRightStartVertex;
	Vertex mRightGoalVertex;

	/// Set of vertices to be rewired.
	unorderedSet mSetRewire;

	/// CompositeEdge evaluation resolution manager.
	utils::BisectPerm mBisectPermObj;

	///////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////

	// Supplementary Functions
	/// Given a new edge, initialize the embedded configurations
	/// along the edge using the resolution of the underlying space.
	/// This is called during problem setup.
	/// \param[in] e The edge ID to initialize with configurations
	void initializeEdgePoints(const CompositeEdge& e);

	/// Evaluate an edge to determine its collision status
	/// and assign it to the underlying property of the edge.
	/// \param[in] The edge ID to check for
	/// \return True if edge is valid
	bool evaluateEdge(const CompositeEdge& e);

	/// Return the edge between the two vertices
	/// \param[in] source Source vertex
	/// \param[in] target Target vertex
	CompositeEdge getEdge(CompositeVertex u, CompositeVertex v) const;

	/// Returns g-value of vertex.
	/// \param[in] vertex CompositeVertex
	double estimateCostToCome(CompositeVertex vertex) const;

	/// Returns heuristic value of vertex.
	/// \param[in] vertex CompositeVertex
	double heuristicFunction(CompositeVertex vertex) const;

	/// Returns f-value of vertex.
	/// \param vertex CompositeVertex
	double estimateTotalCost(CompositeVertex vertex) const;

	/// Construct the solution.
	/// \param[in] start Start vertex
	/// \param[in] goal Goal vertex
	ompl::base::PathPtr constructSolution(const CompositeVertex& start, const CompositeVertex& goal) const;

	/// Get the neighbors of a CompositeVertex in composite_map
	/// \param[in] vertex
	std::vector<CompositeVertex> getNeighbors(CompositeVertex &v);

	/// A Star
	std::vector<CompositeVertex> AStar();

	std::vector<Vertex> leftComputeShortestPath(std::vector<Constraint> constraints, double& costOut);
	std::vector<Vertex> leftLazyComputeShortestPath(std::vector<Constraint> constraints, double& costOut);
	std::vector<Vertex> rightComputeShortestPath(std::vector<Constraint> constraints, double& costOut);
	std::vector<Vertex> rightLazyComputeShortestPath(std::vector<Constraint> constraints, double& costOut);
}; // class LazyCBS

} // namespace LazyCBS


namespace LazyCBS
{

LazyCBS::LazyCBS(const ompl::base::SpaceInformationPtr &si)
	: ompl::base::Planner(si, "LazyCBS")
	, mSpace(si->getStateSpace())
	, mLeftRoadmapFileName("")
	, mRightRoadmapFileName("")
	, mConnectionRadius(10.0)
	, mCheckRadius(0.005*mConnectionRadius)
	, mTPG(si->getStateSpace())
{
	// Register my setting callbacks.
	Planner::declareParam<std::string>("leftRoadmapFilename", this, &LazyCBS::setLeftRoadmapFileName, &LazyCBS::getLeftRoadmapFileName);
	Planner::declareParam<std::string>("rightRoadmapFilename", this, &LazyCBS::setRightRoadmapFileName, &LazyCBS::getRightRoadmapFileName);
}

LazyCBS::LazyCBS(const ompl::base::SpaceInformationPtr &si,
	const std::string& leftRoadmapFileName, const std::string& rightRoadmapFileName)
	: ompl::base::Planner(si, "LazyCBS")
	, mSpace(si->getStateSpace())
	, mLeftRoadmapFileName(leftRoadmapFileName)
	, mRightRoadmapFileName(rightRoadmapFileName)
	, mConnectionRadius(4.0)
	, mCheckRadius(0.005*mConnectionRadius)
	, mTPG(si->getStateSpace())
{

	if (mLeftRoadmapFileName == "")
		throw std::invalid_argument("Provide a non-empty path to left roadmap.");
	if (mRightRoadmapFileName == "")
		throw std::invalid_argument("Provide a non-empty path to right roadmap.");
}

LazyCBS::~LazyCBS()
{
	// Do nothing.
}

// ===========================================================================================
void LazyCBS::setup()
{
	// Check if already setup.
	if (static_cast<bool>(ompl::base::Planner::setup_))
		return;

	ompl::base::Planner::setup();

	// roadmapPtr = boost::shared_ptr<utils::RoadmapFromFile<CompositeGraph, CompositeVPStateMap, utils::StateWrapper, CompositeEPLengthMap, CompositeEPPriorMap>>
	//               (new utils::RoadmapFromFile<CompositeGraph, CompositeVPStateMap, utils::StateWrapper, CompositeEPLengthMap, CompositeEPPriorMap>
	//               (mSpace, mRoadmapFileName));

	// roadmapPtr->generate(graph,
	//                      get(&CompositeVProp::state, graph),
	//                      get(&CompositeEProp::length, graph),
	//                      get(&CompositeEProp::prior, graph));

	create_vertices(left_graph,get(&VProp::state,left_graph),mLeftRoadmapFileName,mSpace->getDimension()/2,get(&EProp::prior,left_graph));
	create_edges(left_graph,get(&EProp::length,left_graph));

	VertexIter ind_vi, ind_vi_end;
	size_t i=0;
	for (boost::tie(ind_vi, ind_vi_end) = vertices(left_graph); ind_vi != ind_vi_end; ++ind_vi,++i)
	{
		put(&VProp::vertex_index,left_graph,*ind_vi,i);
	}

	create_vertices(right_graph,get(&VProp::state,right_graph),mRightRoadmapFileName,mSpace->getDimension()/2,get(&EProp::prior,right_graph));
	create_edges(right_graph,get(&EProp::length,right_graph));
	i=0;
	for (boost::tie(ind_vi, ind_vi_end) = vertices(right_graph); ind_vi != ind_vi_end; ++ind_vi,++i)
	{
		put(&VProp::vertex_index,right_graph,*ind_vi,i);
	}

	// VertexIter ind_vi, ind_vi_end;

	// std::unordered_map<Eigen::VectorXd, CompositeVertex, matrix_hash<Eigen::VectorXd>> configToNodeMap;

	// for(boost::tie(ind_vi,ind_vi_end) = vertices(individual_graph); ind_vi!=ind_vi_end; ind_vi++)
	// {
	//   Eigen::VectorXd config = individual_graph[*ind_vi].state;
	//   utils::StateWrapperPtr newState(new utils::StateWrapper(mSpace));
	//   ompl::base::State *ver_state{newState->state};
	//   double *values{ver_state->as<ompl::base::RealVectorStateSpace::StateType>()->values};
	//   for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
	//   {
	//     values[ui] = config[ui];
	//   }
		
	//   CompositeVertex newVertex = boost::add_vertex(graph);
	//   graph[newVertex].state = newState;
	//   configToNodeMap[config]=newVertex;
	// }

	// EdgeIter ind_ei, ind_ei_end;
	// for (boost::tie(ind_ei, ind_ei_end) = edges(individual_graph); ind_ei != ind_ei_end; ++ind_ei)
	// {
	//   CompositeVertex source_vertex = configToNodeMap[individual_graph[source(*ind_ei, individual_graph)].state];
	//   CompositeVertex target_vertex = configToNodeMap[individual_graph[target(*ind_ei, individual_graph)].state];
	//   std::pair<CompositeEdge,bool> newEdge = boost::add_edge(source_vertex, target_vertex, graph);
	//   graph[newEdge.first].length = mSpace->distance(graph[source_vertex].state->state, graph[target_vertex].state->state);
	//   graph[newEdge.first].prior = 1.0;
	// }
	// // Set default vertex values.
	// CompositeVertexIter vi, vi_end;
	// for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
	// {
	//   graph[*vi].distance = std::numeric_limits<double>::infinity();
	//   graph[*vi].heuristic = std::numeric_limits<double>::infinity();
	//   graph[*vi].visited = false;
	//   graph[*vi].status = CollisionStatus::FREE;
	// }

	// // Set default edge values.
	// CompositeEdgeIter ei, ei_end;
	// for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei)
	// {
	//   graph[*ei].isEvaluated = false;
	//   graph[*ei].status = CollisionStatus::FREE;
	// }

	mBestPathCost = std::numeric_limits<double>::infinity();
}

// ===========================================================================================
void LazyCBS::clear()
{
	// Set default vertex values.
	CompositeVertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
	{
		graph[*vi].distance = std::numeric_limits<double>::infinity();
		graph[*vi].heuristic = std::numeric_limits<double>::infinity();
		graph[*vi].visited = false;
		graph[*vi].status = CollisionStatus::FREE;
	}

	// Set default edge values.
	CompositeEdgeIter ei, ei_end;
	for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei)
	{
		graph[*ei].isEvaluated = false;
		graph[*ei].status = CollisionStatus::FREE;
	}

	// Internal evaluation variables.
	mBestPathCost = std::numeric_limits<double>::infinity();
	mNumEdgeEvals = 0;
	mNumEdgeRewires = 0;

	// Helper containers.
	mSetRewire.clear();
}

// ===========================================================================================
void LazyCBS::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
{
	// Make sure we setup the planner first.
	if (!static_cast<bool>(ompl::base::Planner::setup_))
	{
		setup();
	}

	ompl::base::Planner::setProblemDefinition(pdef);

	utils::StateWrapperPtr startState(new utils::StateWrapper(mSpace));
	mSpace->copyState(startState->state, pdef_->getStartState(0));

	utils::StateWrapperPtr goalState(new utils::StateWrapper(mSpace));
	mSpace->copyState(goalState->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState());

	auto validityChecker = si_->getStateValidityChecker();

	selector = std::string("composite"); // global selector
	if(!validityChecker->isValid(startState->state))
		throw ompl::Exception("Start configuration is in collision!");
	selector = std::string("composite"); // global selector
	if(!validityChecker->isValid(goalState->state))
		throw ompl::Exception("Goal configuration is in collision!");

	double* startValues = startState->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  

	// std::cout<<startValues[0]<<std::endl<<startValues[1]<<std::endl<<startValues[2]
	// 	<<std::endl<<startValues[3]<<std::endl<<startValues[4]<<std::endl<<startValues[5]
	// 	<<std::endl<<startValues[6]<<std::endl;
	Eigen::VectorXd left_start_config(mSpace->getDimension()/2);
	for (size_t ui = 0; ui < mSpace->getDimension()/2; ui++)
	{
		left_start_config[ui] = startValues[ui];
	}
	// left_start_config << startValues[0], startValues[1], startValues[2], startValues[3],
	// 	startValues[4], startValues[5], startValues[6];

	Eigen::VectorXd right_start_config(mSpace->getDimension()/2);
	for (size_t ui = mSpace->getDimension()/2; ui < mSpace->getDimension(); ui++)
	{
		right_start_config[ui-mSpace->getDimension()/2] = startValues[ui];
	}
	// right_start_config << startValues[7], startValues[8], startValues[9], startValues[10],
		// startValues[11], startValues[12], startValues[13];

	double* goalValues = goalState->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  

	Eigen::VectorXd left_goal_config(mSpace->getDimension()/2);
	for (size_t ui = 0; ui < mSpace->getDimension()/2; ui++)
	{
		left_goal_config[ui] = goalValues[ui];
	}
	// left_goal_config << goalValues[0], goalValues[1], goalValues[2], goalValues[3],
	// 	goalValues[4], goalValues[5], goalValues[6];

	Eigen::VectorXd right_goal_config(mSpace->getDimension()/2);
	for (size_t ui = mSpace->getDimension()/2; ui < mSpace->getDimension(); ui++)
	{
		right_goal_config[ui-mSpace->getDimension()/2] = goalValues[ui];
	}
	// right_goal_config << goalValues[7], goalValues[8], goalValues[9], goalValues[10],
	// 	goalValues[11], goalValues[12], goalValues[13];

	// start and goal added to left graph 

	// std::cout<<left_start_config<<std::endl;

	// Vertex mLeftStartVertex;
	mLeftStartVertex = add_vertex(left_graph);
	left_graph[mLeftStartVertex].state = left_start_config;
	left_graph[mLeftStartVertex].vertex_index = boost::num_vertices(left_graph) - 1;

	// std::cout<<left_graph[mLeftStartVertex].state<<std::endl;

	// Vertex mLeftGoalVertex;
	mLeftGoalVertex = add_vertex(left_graph);
	left_graph[mLeftGoalVertex].state = left_goal_config;
	left_graph[mLeftGoalVertex].vertex_index = boost::num_vertices(left_graph) - 1;

	size_t startDegree = 0;
	size_t goalDegree = 0;

	VertexIter ind_vi, ind_vi_end;
	for (boost::tie(ind_vi, ind_vi_end) = vertices(left_graph); ind_vi != ind_vi_end; ++ind_vi)
	{
		double startDist = (left_graph[mLeftStartVertex].state-left_graph[*ind_vi].state).norm();
		double goalDist = (left_graph[mLeftGoalVertex].state-left_graph[*ind_vi].state).norm();

		if (startDist < mConnectionRadius)
		{
			if(mLeftStartVertex == *ind_vi)
				continue;
			std::pair<Edge,bool> newEdge = boost::add_edge(mLeftStartVertex, *ind_vi, left_graph);
			left_graph[newEdge.first].length = startDist;
			left_graph[newEdge.first].prior = 1.0;
			left_graph[newEdge.first].isEvaluated = false;
			left_graph[newEdge.first].status = CollisionStatus::FREE;
			startDegree++;
		}

		if (goalDist < mConnectionRadius)
		{
			if(mLeftGoalVertex == *ind_vi)
				continue;
			std::pair<Edge,bool> newEdge = boost::add_edge(mLeftGoalVertex, *ind_vi, left_graph);
			left_graph[newEdge.first].length = goalDist;
			left_graph[newEdge.first].prior = 1.0;
			left_graph[newEdge.first].isEvaluated = false;
			left_graph[newEdge.first].status = CollisionStatus::FREE;
			goalDegree++;
		}
	}

	// std::cout<<"Left Graph: Start Degree: "<<startDegree<<" Goal Degree: "<<goalDegree<<std::endl;


	// start and goal added to right graph 

	// Vertex mRightStartVertex;
	mRightStartVertex = add_vertex(right_graph);
	right_graph[mRightStartVertex].state = right_start_config;
	right_graph[mRightStartVertex].vertex_index = boost::num_vertices(right_graph) - 1;

	// Vertex mRightGoalVertex;
	mRightGoalVertex = add_vertex(right_graph);
	right_graph[mRightGoalVertex].state = right_goal_config;
	right_graph[mRightGoalVertex].vertex_index = boost::num_vertices(right_graph) - 1;

	startDegree = 0;
	goalDegree = 0;

	// VertexIter ind_vi, ind_vi_end;
	for (boost::tie(ind_vi, ind_vi_end) = vertices(right_graph); ind_vi != ind_vi_end; ++ind_vi)
	{
		double startDist = (right_graph[mRightStartVertex].state-right_graph[*ind_vi].state).norm();
		double goalDist = (right_graph[mRightGoalVertex].state-right_graph[*ind_vi].state).norm();

		if (startDist < mConnectionRadius)
		{
			if(mRightStartVertex == *ind_vi)
				continue;
			std::pair<Edge,bool> newEdge = boost::add_edge(mRightStartVertex, *ind_vi, right_graph);
			right_graph[newEdge.first].length = startDist;
			right_graph[newEdge.first].prior = 1.0;
			right_graph[newEdge.first].isEvaluated = false;
			right_graph[newEdge.first].status = CollisionStatus::FREE;
			startDegree++;
		}

		if (goalDist < mConnectionRadius)
		{
			if(mRightGoalVertex == *ind_vi)
				continue;
			std::pair<Edge,bool> newEdge = boost::add_edge(mRightGoalVertex, *ind_vi, right_graph);
			right_graph[newEdge.first].length = goalDist;
			right_graph[newEdge.first].prior = 1.0;
			right_graph[newEdge.first].isEvaluated = false;
			right_graph[newEdge.first].status = CollisionStatus::FREE;
			goalDegree++;
		}
	}

	// std::cout<<"Right Graph: Start Degree: "<<startDegree<<" Goal Degree: "<<goalDegree<<std::endl;
	
	// Add start and goal vertices to the graph
	mStartVertex = boost::add_vertex(graph);
	graph[mStartVertex].state = startState;
	graph[mStartVertex].vertex_index = boost::num_vertices(graph) - 1;

	mGoalVertex = boost::add_vertex(graph);
	graph[mGoalVertex].state = goalState;
	graph[mGoalVertex].vertex_index = boost::num_vertices(graph) - 1;

	// Assign default values
	graph[mStartVertex].distance = 0;
	graph[mStartVertex].heuristic = std::max((left_start_config- left_goal_config).norm(),
		(right_start_config - right_goal_config).norm()) ;
	graph[mStartVertex].parent = -1;
	graph[mStartVertex].visited = false;
	graph[mStartVertex].status = CollisionStatus::FREE;

	graph[mGoalVertex].distance = std::numeric_limits<double>::infinity();
	graph[mGoalVertex].heuristic = 0;
	graph[mGoalVertex].visited = false;
	graph[mGoalVertex].status = CollisionStatus::FREE;

	// std::cout<<"DEBUG:"<<left_graph[mLeftStartVertex].vertex_index<<std::endl;
	// std::cout<<left_graph[mLeftStartVertex].state<<std::endl;
	// Populate the tensor product generator class
	mTPG.populateMaps(left_graph,right_graph,graph,mStartVertex,mGoalVertex);

	// getNeighbors(mStartVertex);
	// std::cout<<"Left Edge Status: "<<edge(mLeftStartVertex, mLeftGoalVertex , left_graph).second<<std::endl;
	// std::cout<<"Right Edge Status: "<<edge(mRightStartVertex, mRightGoalVertex , right_graph).second<<std::endl;
	// std::cout<<"Composite Edge Status: "<<edge(mStartVertex, mGoalVertex , graph).second<<std::endl;

	// size_t countval=0;
	// std::cout<<"Nodes connected to left start vertex: ";
	// NeighborIter ai, aiend;
	// for (boost::tie(ai, aiend) = adjacent_vertices(mLeftStartVertex, left_graph); ai != aiend; ++ai) 
	// {
	// 	std::cout<<left_graph[*ai].vertex_index<<" ";
	// 	countval++;
	// }
	// std::cout<<std::endl;
	// std::cout<<countval<<std::endl;
	// countval=0;
	// std::cout<<"Nodes connected to right start vertex: ";
	// for (boost::tie(ai, aiend) = adjacent_vertices(mRightStartVertex, right_graph); ai != aiend; ++ai) 
	// {
	// 	std::cout<<right_graph[*ai].vertex_index<<" ";
	// 	countval++;
	// }
	// std::cout<<std::endl;
	// std::cout<<countval<<std::endl;
	// countval=0;
	// std::cout<<"Nodes connected to composite start vertex: ";
	// CompositeNeighborIter aic, aiendc;
	// for (boost::tie(aic, aiendc) = adjacent_vertices(mStartVertex, graph); aic != aiendc; ++aic) 
	// {
	// 	std::cout<<graph[*aic].vertex_index<<" ";
	// 	countval++;
	// }
	// std::cout<<std::endl;
	// std::cout<<countval<<std::endl;

	// std::cin.get();
	// ///explicit
	// mTPG.explicitTPG(left_graph,mSpace->getDimension()/2,right_graph,mSpace->getDimension()/2,graph, left_goal_config, right_goal_config);

	// std::cout<<"Press [ENTER] to continue:";
	// std::cin.get();
	// Eigen::VectorXd start_config(mSpace->getDimension());
	// Eigen::VectorXd goal_config(mSpace->getDimension());

	// start_config << left_start_config, right_start_config;
	// goal_config << left_goal_config, right_goal_config;

	// mStartVertex = mTPG.configToNodeTPG(start_config);
	// mGoalVertex = mTPG.configToNodeTPG(goal_config);
}

std::vector<CompositeVertex> LazyCBS::getNeighbors(CompositeVertex &v)
{
	std::vector <CompositeVertex> neighbors;
	Eigen::VectorXd goal_config(mSpace->getDimension());
	double* goal_values = graph[mGoalVertex].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
	for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
	{
		goal_config[ui] = goal_values[ui];
	}

	// goal_config << goal_values[0], goal_values[1], goal_values[2], goal_values[3], goal_values[4],
	// 	goal_values[5], goal_values[6], goal_values[7], goal_values[8], goal_values[9], 
	// 	goal_values[10], goal_values[11], goal_values[12], goal_values[13];

	// std::cout<<"Calling implicit neighbors!"<<std::endl;
	
	neighbors = mTPG.getNeighborsImplicitTPG(v, graph, left_graph, right_graph, goal_config);
	
	// Initialize neighbor values right now being set in BGL_Definitions
	return neighbors;

	// std::vector <CompositeVertex> neighbors;
	// CompositeNeighborIter ai, aiend;
	// for (boost::tie(ai, aiend) = adjacent_vertices(v, graph); ai != aiend; ++ai) 
	// {
	// 	neighbors.push_back(*ai);
	// }
	// return neighbors;
}

std::vector<CompositeVertex> LazyCBS::AStar()
{
	std::chrono::time_point<std::chrono::system_clock> start_time, end_time,
		start_neighbors_time, end_neighbors_time, start_queue_time, end_queue_time,
		start_neighbors_manip_time, end_neighbors_manip_time,
		start_get_edge_time, end_get_edge_time; 

	start_time = std::chrono::system_clock::now(); 

	std::chrono::duration<double> elapsed_seconds;
	std::chrono::duration<double> neighbors_elapsed_seconds;
	std::chrono::duration<double> queue_elapsed_seconds;
	std::chrono::duration<double> neighbors_manip_elapsed_seconds;
	std::chrono::duration<double> get_edge_elapsed_seconds;
	// 	std::cout<<"Press [ENTER] to continue:";
	// std::cin.get();
	// Set default vertex values.
	CompositeVertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
	{
	  graph[*vi].distance = std::numeric_limits<double>::infinity();
	  // graph[*vi].heuristic = std::numeric_limits<double>::infinity();
	  graph[*vi].visited = false;
	  graph[*vi].status = CollisionStatus::FREE;
	}

	graph[mStartVertex].distance = 0;
	// graph[mStartVertex].heuristic = std::max((left_start_config- left_goal_config).norm(),
	// 	(right_start_config - right_goal_config).norm()) ;
	graph[mStartVertex].parent = -1;

	// graph[mGoalVertex].heuristic = 0;

	// Priority Function: f-value
	auto cmpFValue = [&](const CompositeVertex& left, const CompositeVertex& right)
	{
		double estimateLeft = estimateTotalCost(left);
		double estimateRight = estimateTotalCost(right);

		if (estimateRight - estimateLeft > 0)
			return true;
		if (estimateLeft - estimateRight > 0)
			return false;
		if (left < right)
			return true;
		else
			return false;
	};
	// std::cout<<"Press [ENTER] to continue:";
	// std::cin.get();

	std::set<CompositeVertex, decltype(cmpFValue)> qUseful(cmpFValue);

	bool solutionFound = false;

	graph[mStartVertex].visited = true;
	qUseful.insert(mStartVertex);

	// std::cout<<"Press [ENTER] to continue:";
	// std::cin.get();

	size_t iteration=0;

	size_t num_of_neighbors=0;
	
	int get_edge_flag=1;

	while(qUseful.size()!=0)
	{
		iteration++;
		start_queue_time = std::chrono::system_clock::now(); 
		CompositeVertex vTop = *qUseful.begin();
		qUseful.erase(qUseful.begin());
		end_queue_time = std::chrono::system_clock::now(); 
		if(iteration==1)
			queue_elapsed_seconds = (end_queue_time - start_queue_time);
		else
			queue_elapsed_seconds += (end_queue_time - start_queue_time);
		
		if(vTop == mGoalVertex)
		{
			solutionFound = true;
			break;      
		}

		start_neighbors_time = std::chrono::system_clock::now(); 
		std::vector <CompositeVertex> neighbors = getNeighbors(vTop);
		end_neighbors_time = std::chrono::system_clock::now(); 

		if(iteration==1)
			neighbors_elapsed_seconds = (end_neighbors_time - start_neighbors_time);
		else
			neighbors_elapsed_seconds += (end_neighbors_time - start_neighbors_time);

		start_neighbors_manip_time = std::chrono::system_clock::now(); 
		for (auto ai = neighbors.begin() ; ai != neighbors.end(); ++ai) 
		{
			// displayGraph(graph);
			CompositeVertex successor = *ai; 
			CompositeEdge uv = getEdge(successor,vTop);
			double edgeLength = graph[uv].length;
			double new_cost = graph[vTop].distance + edgeLength;
				// std::cout<<"Edge is Free!"<<std::endl; 
			if(new_cost < graph[successor].distance)
			{
			 graph[successor].distance = new_cost;
			start_get_edge_time = std::chrono::system_clock::now();
			 qUseful.insert(successor);
			end_get_edge_time = std::chrono::system_clock::now();
			 graph[successor].parent= vTop;
			if(get_edge_flag==1)
			{
				get_edge_flag=0;
				get_edge_elapsed_seconds = (end_get_edge_time - start_get_edge_time);
			}
			else
				get_edge_elapsed_seconds += (end_get_edge_time - start_get_edge_time);
			}
			num_of_neighbors++;
		}
		end_neighbors_manip_time = std::chrono::system_clock::now(); 

		if(iteration==1)
			neighbors_manip_elapsed_seconds = (end_neighbors_manip_time - start_neighbors_manip_time);
		else
			neighbors_manip_elapsed_seconds += (end_neighbors_manip_time - start_neighbors_manip_time);
	}
	if (graph[mGoalVertex].distance == std::numeric_limits<double>::infinity())
		return std::vector<CompositeVertex>();

	// std::cout<<"Press [ENTER] to continue:";
	// std::cin.get();

	std::vector<CompositeVertex> path;
	
	CompositeVertex node = mGoalVertex;
	
	while(node!=mStartVertex)
	{
		path.push_back(node);
		node=graph[node].parent;
	}
	path.push_back(mStartVertex);
	std::reverse(path.begin(), path.end());
	end_time = std::chrono::system_clock::now(); 
	elapsed_seconds = (end_time - start_time);
	// std::cout<<" Number of Iterations: "<<iteration<<std::endl;
	// std::cout<<" Number of neighbors: "<<num_of_neighbors<<std::endl;
    // std::cout<< "Time taken for iteration: " << elapsed_seconds.count() << "s\n"; 
 //    std::cout<< "Time taken for getNeighbors: "<<neighbors_elapsed_seconds.count()<<"s\n";
 //    std::cout<< "Time taken for queue: " << queue_elapsed_seconds.count() << "s\n"; 
 //    std::cout<< "Time taken for neighbors manipulation: " << neighbors_manip_elapsed_seconds.count() << "s\n"; 
 //    std::cout<< "Time taken for get_edge calls: " << get_edge_elapsed_seconds.count() << "s\n"; 
	return path;
}

// ===========================================================================================
ompl::base::PlannerStatus LazyCBS::solve(const ompl::base::PlannerTerminationCondition & ptc)
{
	std::cout<<"IN SOLVE:!"<<std::endl;
	bool solutionFound = false;

	// Log Time
	std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};

	size_t numSearches=0;
	while(!solutionFound)
	{
		// std::cout<<"Search: "<<++numSearches<<std::endl;
		std::vector<CompositeVertex> shortestPath = AStar();
		// std::cout<<"Graph Size- #Vertices: "<<num_vertices(graph)<<" #Edges: "<<num_edges(graph)<<std::endl;
		// double pathLength=0;
		// for(size_t i=0; i<shortestPath.size()-1; i++)
		// {
		// 	CompositeEdge uv = getEdge(shortestPath.at(i),shortestPath.at(i+1));			
		// 	pathLength+=graph[uv].length;
		// }
		// std::cout<<"Path Length: "<<pathLength<<" ";
		// std::cout<<"Path: ";
		// for(Vertex &nodes: shortestPath )
		// 	std::cout<<graph[nodes].vertex_index<<" ";
		// std::cout<<std::endl<<std::endl;
		// displayPath(graph,shortestPath);
		if(shortestPath.size()==0)
			break;

		solutionFound = true;
		for(size_t i=0; i<shortestPath.size()-1; i++)
		{
			CompositeEdge uv = getEdge(shortestPath.at(i),shortestPath.at(i+1));			
			if(!graph[uv].isEvaluated)
			{
				graph[uv].isEvaluated = true;
				if(!evaluateEdge(uv))
				{
					// std::cout<<"Edge between"<<graph[shortestPath.at(i)].vertex_index<<" and "
					// 	<<graph[shortestPath.at(i+1)].vertex_index<<" is in collision."<<std::endl;
					graph[uv].length = std::numeric_limits<double>::infinity();
					solutionFound = false;
					break;
				}
			}
		}
	}
	// std::vector<CompositeVertex> shortestPath = AStar();
	// if(shortestPath.size()==0)
	// 	OMPL_INFORM("Solution NOT PRESENT!");

	std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
	std::chrono::duration<double> elapsedSeconds{endTime-startTime};
	mSearchTime = elapsedSeconds.count() - mEdgeEvaluationsTime - mLogTime;

	if(solutionFound)
	{
		OMPL_INFORM("Solution Found!");
		mBestPathCost = estimateCostToCome(mGoalVertex);
		pdef_->addSolutionPath(constructSolution(mStartVertex, mGoalVertex));	
		OMPL_INFORM("Graph Size:     #Vertices:%d        #Edges:%d",num_vertices(graph),num_edges(graph));
		OMPL_INFORM("Search Time:                 %f", elapsedSeconds.count());
		OMPL_INFORM("Number of Iterations:        %d", numSearches);	
		OMPL_INFORM("Number of Edges Evaluated:   %d", mNumEdgeEvals);
		OMPL_INFORM("Cost of goal:                %f", mBestPathCost);

		return ompl::base::PlannerStatus::EXACT_SOLUTION;
	}

	else
	{
		OMPL_INFORM("Solution NOT Found");
		displayGraph(graph);
	}
}
// ===========================================================================================
// Setters

void LazyCBS::setLeftRoadmapFileName(const std::string& leftRoadmapFileName)
{
	mLeftRoadmapFileName = leftRoadmapFileName;
}

void LazyCBS::setRightRoadmapFileName(const std::string& rightRoadmapFileName)
{
	mRightRoadmapFileName = rightRoadmapFileName;
}


void LazyCBS::setConnectionRadius(double connectionRadius)
{
	mConnectionRadius = connectionRadius;
}

void LazyCBS::setCheckRadius(double checkRadius)
{
	mCheckRadius = checkRadius;
}
// ===========================================================================================
// Getters

std::string LazyCBS::getLeftRoadmapFileName() const
{
	return mLeftRoadmapFileName;
}
std::string LazyCBS::getRightRoadmapFileName() const
{
	return mRightRoadmapFileName;
}

double LazyCBS::getConnectionRadius() const
{
	return mConnectionRadius;
}

double LazyCBS::getCheckRadius() const
{
	return mCheckRadius;
}

CompositeVertex LazyCBS::getStartVertex() const
{
	return mStartVertex;
}

CompositeVertex LazyCBS::getGoalVertex() const
{
	return mGoalVertex;
}

double LazyCBS::getBestPathCost() const
{
	return mBestPathCost;
}

std::size_t LazyCBS::getNumEdgeEvaluations() const
{
	return mNumEdgeEvals;
}

std::size_t LazyCBS::getNumEdgeRewires() const
{
	return mNumEdgeRewires;
}

double LazyCBS::getEdgeEvaluationsTime() const
{
	return mEdgeEvaluationsTime;
}

double LazyCBS::getSearchTime() const
{
	return mSearchTime;
}

// ===========================================================================================
ompl::base::PathPtr LazyCBS::constructSolution(const CompositeVertex &start, const CompositeVertex &goal) const
{
	std::set<CompositeVertex> seen;

	ompl::geometric::PathGeometric *path = new ompl::geometric::PathGeometric(si_);
	CompositeVertex v = goal;
	while (v != start)
	{
		if (seen.find(v) != seen.end())
		{
			OMPL_ERROR("infinite loop");
			break;
		}

		seen.insert(v);
		path->append(graph[v].state->state);
		v = graph[v].parent;
	}

	if (v == start)
	{
		path->append(graph[start].state->state);
	}
	path->reverse();
	return ompl::base::PathPtr(path);
}

// ===========================================================================================
void LazyCBS::initializeEdgePoints(const CompositeEdge& e)
{
	auto startState = graph[source(e,graph)].state->state;
	auto endState = graph[target(e,graph)].state->state;

	unsigned int nStates = static_cast<unsigned int>(std::floor(graph[e].length / (2.0*mCheckRadius)));
	
	// Just start and goal
	if(nStates < 2u)
	{
		nStates = 2u;
	}

	graph[e].edgeStates.resize(nStates);

	for(unsigned int i = 0; i < nStates; i++)
	{
		graph[e].edgeStates[i].reset(new utils::StateWrapper(mSpace));
	}

	const std::vector<std::pair<int,int>>& order = mBisectPermObj.get(nStates);

	for(unsigned int i = 0; i < nStates; i++)
	{
		mSpace->interpolate(startState, endState,
			1.0*(1+order[i].first)/(nStates+1), graph[e].edgeStates[i]->state);
	}
}

// ===========================================================================================
bool LazyCBS::evaluateEdge(const CompositeEdge& e)
{
	// Log Time
	std::chrono::time_point<std::chrono::system_clock> startEvaluationTime{std::chrono::system_clock::now()};

	// Initialize edge states [just in time]. Useful for large graphs.
	initializeEdgePoints(e);

	// March along edge states with highest resolution
	mNumEdgeEvals++;
	graph[e].isEvaluated = true;

	auto validityChecker = si_->getStateValidityChecker();
	
	CompositeVertex startVertex = source(e,graph);
	CompositeVertex endVertex   = target(e,graph);
	auto startState = graph[startVertex].state->state;
	auto endState = graph[endVertex].state->state;

	auto nStates = graph[e].edgeStates.size();

	bool checkResult = true;

	selector = std::string("composite"); // global selector
	
	// Evaluate Start and End States [we only assume states in self-collision are pruned out]
	if (checkResult && !validityChecker->isValid(startState))
	{
		graph[startVertex].status = CollisionStatus::BLOCKED;
		graph[e].status = CollisionStatus::BLOCKED;
		graph[e].length = std::numeric_limits<double>::max();
		checkResult = false;
	}

	selector = std::string("composite"); // global selector

	if (checkResult && !validityChecker->isValid(endState))
	{
		graph[endVertex].status = CollisionStatus::BLOCKED;
		graph[e].status = CollisionStatus::BLOCKED;
		graph[e].length = std::numeric_limits<double>::max();
		checkResult = false;
	}

	if (checkResult)
	{
		// Evaluate the States in between
		for (unsigned int i = 1; i < nStates-1; i++)
		{
			selector = std::string("composite"); // global selector
			if(!validityChecker->isValid(graph[e].edgeStates[i]->state))
			{
				graph[e].status = CollisionStatus::BLOCKED;
				graph[e].length = std::numeric_limits<double>::max();
				checkResult = false;
				break;
			}
		}
	}

	std::chrono::time_point<std::chrono::system_clock> endEvaluationTime{std::chrono::system_clock::now()};
	std::chrono::duration<double> elapsedSeconds{endEvaluationTime-startEvaluationTime};
	mEdgeEvaluationsTime += elapsedSeconds.count();

	return checkResult;
}

// ===========================================================================================
CompositeEdge LazyCBS::getEdge(CompositeVertex u, CompositeVertex v) const
{
	CompositeEdge uv;
	bool edgeExists;
	boost::tie(uv, edgeExists) = edge(u, v, graph);

	return uv;
}

// ===========================================================================================
double LazyCBS::estimateCostToCome(CompositeVertex v) const
{
	// return graph[v].distance + graph[v].lazyCostToCome;
	return graph[v].distance;
}

double LazyCBS::heuristicFunction(CompositeVertex v) const
{
	return graph[v].heuristic;
	// return mSpace->distance(graph[v].state->state, graph[mGoalVertex].state->state);
	// double* config_values = 
	// 	graph[v].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

	// Eigen::VectorXd composite_config(14);
	// composite_config << config_values[0],config_values[1],config_values[2],config_values[3],
	// 	config_values[4],config_values[5],config_values[6],config_values[7],
	// 	config_values[8],config_values[9],config_values[10],config_values[11],
	// 	config_values[12],config_values[13];

	// double* goal_config_values = 
	// 	graph[mGoalVertex].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

	// Eigen::VectorXd goal_composite_config(14);
	// goal_composite_config << goal_config_values[0],goal_config_values[1],goal_config_values[2],goal_config_values[3],
	// 	goal_config_values[4],goal_config_values[5],goal_config_values[6],goal_config_values[7],
	// 	goal_config_values[8],goal_config_values[9],goal_config_values[10],goal_config_values[11],
	// 	goal_config_values[12],goal_config_values[13];
	// return std::max((composite_config.segment(0,7)-goal_composite_config.segment(7,7)).norm(),
	// 						(composite_config.segment(7,7)-goal_composite_config.segment(7,7)).norm());
}

double LazyCBS::estimateTotalCost(CompositeVertex v) const
{
	return estimateCostToCome(v) + heuristicFunction(v);
}

std::vector<Vertex> LazyCBS::leftComputeShortestPath(std::vector<Constraint> constraints, double& costOut)
{
	timePriorityQueue pq;
	std::unordered_map<std::pair<Vertex, size_t>, double, pair_hash> distanceMap;
	std::unordered_map<std::pair<Vertex, size_t> , std::pair<Vertex, size_t> , pair_hash > parentMap;
	std::unordered_map<size_t , Vertex> nodeMap;

	pq.insert(left_graph[mLeftStartVertex].vertex_index,0,0.0,0.0);
	nodeMap[left_graph[mLeftStartVertex].vertex_index]=mLeftStartVertex;

	VertexIter vi, viend;
	distanceMap[std::make_pair(mLeftStartVertex,0)]=0;

	size_t numSearches = 0;
	size_t maximum_timestep = 10000;
	while(pq.PQsize()!=0)
	{
		numSearches++;
		// std::cout<<"Queue pop no: "<<numSearches<<std::endl;
		std::pair<int,size_t> top_element = pq.pop();
		size_t index = top_element.first;
		size_t timestep = top_element.second;
		// std::cout<<"Index: "<<index<<" Timestep: "<<timestep<<" Distance: "<<distanceMap[std::make_pair(nodeMap[index],timestep)] <<std::endl;
		// if(parentMap.count(std::make_pair(nodeMap[index],timestep)))
			// std::cout<<"Parent: "<<indexMap[parentMap[std::make_pair(nodeMap[index],timestep)].first] <<std::endl;
		if(timestep > maximum_timestep)
			break;
		if(index == left_graph[mLeftGoalVertex].vertex_index)
			break;
		Vertex curr_node = nodeMap[index];

		std::vector<Vertex> neighbors;
		NeighborIter ai;
		NeighborIter aiEnd;

		for (tie(ai, aiEnd) = adjacent_vertices(curr_node, left_graph); ai != aiEnd; ++ai) 
		{
			Vertex curNeighbor = *ai;
			neighbors.push_back(curNeighbor);
		}

		neighbors.push_back(curr_node);
		// std::cout<<"No. of neighbors :"<<neighbors.size()<<std::endl;

		for (auto successor : neighbors) 
		{
			// std::cout<<"Successor: "<<indexMap[successor]<<std::endl;
			if(successor == curr_node)
			{
				double new_cost = distanceMap[std::make_pair(curr_node,timestep)] + PAUSE_PENALTY;
				
				if(distanceMap.count(std::make_pair(successor,timestep+1))==0 || 
					new_cost < distanceMap[std::make_pair(successor,timestep+1)])
				{
					distanceMap[std::make_pair(successor,timestep+1)]= new_cost;
					double priority;
					// std::cout<<"FOUND FREE EDGE!!"<<std::endl;
					priority = new_cost + (left_graph[successor].state-left_graph[mLeftGoalVertex].state).norm();
					pq.insert(left_graph[successor].vertex_index,timestep+1,priority,0.0);
					if(nodeMap.count(left_graph[successor].vertex_index)==0)
						nodeMap[left_graph[successor].vertex_index]=successor;
					parentMap[std::make_pair(successor,timestep+1)]=std::make_pair(curr_node,timestep);
				}
			}
			else
			{
				Edge uv_edge = boost::edge(successor, curr_node, left_graph).first;

				bool col=false;
				for( Constraint c: constraints)
				{
					if( ((left_graph[boost::source(c.e,left_graph)].vertex_index==left_graph[boost::source(uv_edge,left_graph)].vertex_index 
							&& left_graph[boost::target(c.e,left_graph)].vertex_index==left_graph[boost::target(uv_edge,left_graph)].vertex_index) 
						|| (left_graph[boost::source(c.e,left_graph)].vertex_index==left_graph[boost::target(uv_edge,left_graph)].vertex_index 
							&& left_graph[boost::target(c.e,left_graph)].vertex_index==left_graph[boost::source(uv_edge,left_graph)].vertex_index)) 
						&& c.t == timestep)
					{
						std::cout<<"Constraint Encountered! "<<std::endl;
						col =true;
						break;
					}
					// else
						// std::cout<<" No constraints!"<<timestep<<std::endl;
				}

				// std::cout<<"Col: "<<col<<std::endl;
				if(!col)
				{					
					double new_cost = distanceMap[std::make_pair(curr_node,timestep)] + left_graph[uv_edge].length;
					if(distanceMap.count(std::make_pair(successor,timestep+1))==0 || new_cost < distanceMap[std::make_pair(successor,timestep+1)])
					{
						distanceMap[std::make_pair(successor,timestep+1)]= new_cost;
						double priority;
						// std::cout<<"FOUND FREE EDGE!!"<<std::endl;
						priority = new_cost + (left_graph[successor].state-left_graph[mLeftGoalVertex].state).norm();
						pq.insert(left_graph[successor].vertex_index,timestep+1,priority,0.0);
						if(nodeMap.count(left_graph[successor].vertex_index)==0)
							nodeMap[left_graph[successor].vertex_index]=successor;
						parentMap[std::make_pair(successor,timestep+1)]=std::make_pair(curr_node,timestep);
					}
				}
			}
		}
	}

	costOut = std::numeric_limits<double>::infinity();
	int goal_timestep = -1;
	for(size_t t = 0; t<maximum_timestep; t++)
	{
		if(distanceMap.count(std::make_pair(mLeftGoalVertex,t)))
		{
			if(distanceMap[std::make_pair(mLeftGoalVertex,t)]<costOut)
				goal_timestep = t;
		}
	}
	if(goal_timestep == -1)
	{
		std::cout<<"ALL_COL!"<<std::endl;
		return std::vector<Vertex>();
	}

	costOut = distanceMap[std::make_pair(mLeftGoalVertex,goal_timestep)];

	// std::cout<<"Goal Time: "<<goal_timestep<<std::endl;
	std::vector<Vertex> finalPath;
	Vertex node = mLeftGoalVertex;
	// std::cout<<indexMap[parentMap[std::make_pair(node,goal_timestep)].first]<<std::endl;
	while(node!=mLeftStartVertex)
	{
		// std::cout<<"Goal Time: "<<goal_timestep<<"Index:"<<indexMap[node];
		// std::cin.get();
		// std::cout<<"INF LOOP LOL!";
		finalPath.push_back(node);
		Vertex temp_node = node;
		size_t temp_timestep = goal_timestep;
		node=parentMap[std::make_pair(temp_node,temp_timestep)].first;
		goal_timestep=parentMap[std::make_pair(temp_node,temp_timestep)].second;
	}
	finalPath.push_back(mLeftStartVertex);
	std::reverse(finalPath.begin(), finalPath.end());
	return finalPath;
}

std::vector<Vertex> LazyCBS::leftLazyComputeShortestPath(std::vector<Constraint> constraints, double& costOut)
{

	int numSearches = 0;
	// std::cout<<"Input [ENTER] to start search: ";
	// std::cin.get();
	while(true)
	{
		numSearches++;

		std::cout<< "[INFO]: Search "<<numSearches <<std::endl;
		std::vector<Vertex> shortestPath = leftComputeShortestPath(constraints,costOut);
		for(Vertex nodes: shortestPath )
		 std::cout<<left_graph[nodes].vertex_index<<" ";
		if( shortestPath.size()==0)
		{
			std::cout << "" <<std::endl;
			// std::cout<< "[INFO]: ALL COLL" <<std::endl;
			return std::vector<Vertex>();
		}

		bool collisionFree = true;
		for( int i=0;i<shortestPath.size()-1;i++)
		{
			Vertex curU = shortestPath.at(i);
			Vertex curV = shortestPath.at(i+1);

			Edge curEdge = boost::edge(curU,curV,left_graph).first;

			bool curEval = left_graph[curEdge].isEvaluated;

			// std::cout<<std::endl<<"Eval: "<<curEval;

			if(!curEval)
			{
				left_graph[curEdge].isEvaluated = true;
				bool col= !evaluateLeftEdge(curEdge);

				// std::cout<<" Col: "<<col;
				// bool col =false;

				if(col)
				{
					left_graph[curEdge].length = std::numeric_limits<double>::infinity();
					collisionFree=false;
					break;
				}
			}
		}
		if(collisionFree)
		{
			return shortestPath;
		}
	}
}

std::vector<Vertex> LazyCBS::rightComputeShortestPath(std::vector<Constraint> constraints, double& costOut)
{
	timePriorityQueue pq;
	std::unordered_map<std::pair<Vertex, size_t>, double, pair_hash> distanceMap;
	std::unordered_map<std::pair<Vertex, size_t> , std::pair<Vertex, size_t> , pair_hash > parentMap;
	std::unordered_map<size_t , Vertex> nodeMap;

	pq.insert(right_graph[mRightStartVertex].vertex_index,0,0.0,0.0);
	nodeMap[right_graph[mRightStartVertex].vertex_index]=mRightStartVertex;

	VertexIter vi, viend;
	distanceMap[std::make_pair(mRightStartVertex,0)]=0;

	size_t numSearches = 0;
	size_t maximum_timestep = 10000;
	while(pq.PQsize()!=0)
	{
		numSearches++;
		// std::cout<<"Queue pop no: "<<numSearches<<std::endl;
		std::pair<int,size_t> top_element = pq.pop();
		size_t index = top_element.first;
		size_t timestep = top_element.second;
		// std::cout<<"Index: "<<index<<" Timestep: "<<timestep<<" Distance: "<<distanceMap[std::make_pair(nodeMap[index],timestep)] <<std::endl;
		// if(parentMap.count(std::make_pair(nodeMap[index],timestep)))
			// std::cout<<"Parent: "<<indexMap[parentMap[std::make_pair(nodeMap[index],timestep)].first] <<std::endl;
		if(timestep > maximum_timestep)
			break;
		if(index == right_graph[mRightGoalVertex].vertex_index)
			break;
		Vertex curr_node = nodeMap[index];

		std::vector<Vertex> neighbors;
		NeighborIter ai;
		NeighborIter aiEnd;

		for (tie(ai, aiEnd) = adjacent_vertices(curr_node, right_graph); ai != aiEnd; ++ai) 
		{
			Vertex curNeighbor = *ai;
			neighbors.push_back(curNeighbor);
		}

		neighbors.push_back(curr_node);
		// std::cout<<"No. of neighbors :"<<neighbors.size()<<std::endl;

		for (auto successor : neighbors) 
		{
			// std::cout<<"Successor: "<<indexMap[successor]<<std::endl;
			if(successor == curr_node)
			{
				double new_cost = distanceMap[std::make_pair(curr_node,timestep)] + PAUSE_PENALTY;
				
				if(distanceMap.count(std::make_pair(successor,timestep+1))==0 || 
					new_cost < distanceMap[std::make_pair(successor,timestep+1)])
				{
					distanceMap[std::make_pair(successor,timestep+1)]= new_cost;
					double priority;
					// std::cout<<"FOUND FREE EDGE!!"<<std::endl;
					priority = new_cost + (right_graph[successor].state-right_graph[mRightGoalVertex].state).norm();
					pq.insert(right_graph[successor].vertex_index,timestep+1,priority,0.0);
					if(nodeMap.count(right_graph[successor].vertex_index)==0)
						nodeMap[right_graph[successor].vertex_index]=successor;
					parentMap[std::make_pair(successor,timestep+1)]=std::make_pair(curr_node,timestep);
				}
			}
			else
			{
				Edge uv_edge = boost::edge(successor, curr_node, right_graph).first;

				bool col=false;
				for( Constraint c: constraints)
				{
					if( ((right_graph[boost::source(c.e,right_graph)].vertex_index==right_graph[boost::source(uv_edge,right_graph)].vertex_index 
							&& right_graph[boost::target(c.e,right_graph)].vertex_index==right_graph[boost::target(uv_edge,right_graph)].vertex_index) 
						|| (right_graph[boost::source(c.e,right_graph)].vertex_index==right_graph[boost::target(uv_edge,right_graph)].vertex_index 
							&& right_graph[boost::target(c.e,right_graph)].vertex_index==right_graph[boost::source(uv_edge,right_graph)].vertex_index)) 
						&& c.t == timestep)
					{
						std::cout<<"Constraint Encountered! "<<std::endl;
						col =true;
						break;
					}
					// else
						// std::cout<<" No constraints!"<<timestep<<std::endl;
				}

				// std::cout<<"Col: "<<col<<std::endl;
				if(!col)
				{					
					double new_cost = distanceMap[std::make_pair(curr_node,timestep)] + right_graph[uv_edge].length;
					if(distanceMap.count(std::make_pair(successor,timestep+1))==0 || new_cost < distanceMap[std::make_pair(successor,timestep+1)])
					{
						distanceMap[std::make_pair(successor,timestep+1)]= new_cost;
						double priority;
						// std::cout<<"FOUND FREE EDGE!!"<<std::endl;
						priority = new_cost + (right_graph[successor].state-right_graph[mRightGoalVertex].state).norm();
						pq.insert(right_graph[successor].vertex_index,timestep+1,priority,0.0);
						if(nodeMap.count(right_graph[successor].vertex_index)==0)
							nodeMap[right_graph[successor].vertex_index]=successor;
						parentMap[std::make_pair(successor,timestep+1)]=std::make_pair(curr_node,timestep);
					}
				}
			}
		}
	}

	costOut = std::numeric_limits<double>::infinity();
	int goal_timestep = -1;
	for(size_t t = 0; t<maximum_timestep; t++)
	{
		if(distanceMap.count(std::make_pair(mRightGoalVertex,t)))
		{
			if(distanceMap[std::make_pair(mRightGoalVertex,t)]<costOut)
				goal_timestep = t;
		}
	}
	if(goal_timestep == -1)
	{
		std::cout<<"ALL_COL!"<<std::endl;
		return std::vector<Vertex>();
	}

	costOut = distanceMap[std::make_pair(mRightGoalVertex,goal_timestep)];

	// std::cout<<"Goal Time: "<<goal_timestep<<std::endl;
	std::vector<Vertex> finalPath;
	Vertex node = mRightGoalVertex;
	// std::cout<<indexMap[parentMap[std::make_pair(node,goal_timestep)].first]<<std::endl;
	while(node!=mRightStartVertex)
	{
		// std::cout<<"Goal Time: "<<goal_timestep<<"Index:"<<indexMap[node];
		// std::cin.get();
		// std::cout<<"INF LOOP LOL!";
		finalPath.push_back(node);
		Vertex temp_node = node;
		size_t temp_timestep = goal_timestep;
		node=parentMap[std::make_pair(temp_node,temp_timestep)].first;
		goal_timestep=parentMap[std::make_pair(temp_node,temp_timestep)].second;
	}
	finalPath.push_back(mRightStartVertex);
	std::reverse(finalPath.begin(), finalPath.end());
	return finalPath;
}

std::vector<Vertex> LazyCBS::rightLazyComputeShortestPath(std::vector<Constraint> constraints, double& costOut)
{

	int numSearches = 0;
	// std::cout<<"Input [ENTER] to start search: ";
	// std::cin.get();
	while(true)
	{
		numSearches++;

		std::cout<< "[INFO]: Search "<<numSearches <<std::endl;
		std::vector<Vertex> shortestPath = rightComputeShortestPath(constraints,costOut);
		for(Vertex nodes: shortestPath )
		 std::cout<<right_graph[nodes].vertex_index<<" ";
		if( shortestPath.size()==0)
		{
			std::cout << "" <<std::endl;
			// std::cout<< "[INFO]: ALL COLL" <<std::endl;
			return std::vector<Vertex>();
		}

		bool collisionFree = true;
		for( int i=0;i<shortestPath.size()-1;i++)
		{
			Vertex curU = shortestPath.at(i);
			Vertex curV = shortestPath.at(i+1);

			Edge curEdge = boost::edge(curU,curV,right_graph).first;

			bool curEval = right_graph[curEdge].isEvaluated;

			// std::cout<<std::endl<<"Eval: "<<curEval;

			if(!curEval)
			{
				right_graph[curEdge].isEvaluated = true;
				bool col= !evaluateRightEdge(curEdge);

				// std::cout<<" Col: "<<col;
				// bool col =false;

				if(col)
				{
					right_graph[curEdge].length = std::numeric_limits<double>::infinity();
					collisionFree=false;
					break;
				}
			}
		}
		if(collisionFree)
		{
			return shortestPath;
		}
	}
}

} // namespace LazyCBS

#endif // LazyCBS_LazyCBS_HPP_

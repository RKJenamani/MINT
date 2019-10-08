/* Authors: Rajat Kumar Jenamani */

#ifndef MINT_HPP_
#define MINT_HPP_

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

// MINT defined headers
#include "util.hpp"
#include "BGLDefinitions.hpp"
#include "LoadGraphfromFile.hpp"
#include "tensorProductGenerator.hpp"

#define eps 1e-9

// namespace std //As  we are using map with Eigen::VectorXd as key!
// {
//  template<> struct less<Eigen::VectorXd>
//  {
//      bool operator() (Eigen::VectorXd const& a, Eigen::VectorXd const& b) const
//      {
//          assert(a.size()==b.size());
//          for(size_t i=0;i<a.size();++i)
//          {
//              if(a[i]<b[i]) return true;
//              if(a[i]>b[i]) return false;
//          }
//          return false;
//      }
//  };
// }
namespace MINT {

using namespace BGL_DEFINITIONS;

void displayGraph(CompositeGraph &graph)
{
  // Obtain the image matrix
  cv::Mat image = cv::imread("/home/rajat/personalrobotics/ompl_ws/src/MINT/data/obstacles/circle2D.png", 1);
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
	cv::Mat image = cv::imread("/home/rajat/personalrobotics/ompl_ws/src/MINT/data/obstacles/circle2D.png", 1);
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
//  std::size_t operator()(T const& matrix) const {
//      // Note that it is oblivious to the storage order of Eigen matrix (column- or
//      // row-major). It will give you the same hash value for two different matrices if they
//      // are the transpose of each other in different storage order.
//      size_t seed = 0;
//      for (size_t i = 0; i < matrix.size(); ++i) {
//          auto elem = *(matrix.data() + i);
//          seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
//      }
//      return seed;
//  }
// };

/// The OMPL Planner class that implements the algorithm
class MINT: public ompl::base::Planner
{
public:
	/// Constructor
	/// \param[in] si The OMPL space information manager
	explicit MINT(const ompl::base::SpaceInformationPtr &si);

	/// \param[in] si The OMPL space information manager
	/// \param[in] roadmapFileName The path to the .graphml file that encodes the roadmap.
	/// \param[in] greediness The greediness to evaluate lazy shortest paths. Default is 1.
	MINT(const ompl::base::SpaceInformationPtr &si,
					const std::string& roadmapFileName);

	/// Destructor
	~MINT(void);

	///////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////

	/// Unordered set of graph vertices to track the lazy band.
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

	/// Set roadmap information.
	void setRoadmapFileName(const std::string& roadmapFileName);

	/// Set connection radius
	void setConnectionRadius(double connectionRadius);

	/// Set collision checking radius
	void setCheckRadius(double checkRadius);

	/// Get roadmap information.
	std::string getRoadmapFileName() const;

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
	// Priority Function: key-value

	/// Roadmap
	// boost::shared_ptr<utils::RoadmapFromFile<CompositeGraph, CompositeVPStateMap, utils::StateWrapper, CompositeEPLengthMap, CompositeEPPriorMap>> roadmapPtr;

	/// Path to the roadmap.
	std::string mRoadmapFileName;

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

	/// Returns heursitic value of vertex.
	/// \param[in] vertex CompositeVertex
	double heuristicFunction(CompositeVertex vertex) const;

	/// Construct the solution.
	/// \param[in] start Start vertex
	/// \param[in] goal Goal vertex
	ompl::base::PathPtr constructSolution(const CompositeVertex& start, const CompositeVertex& goal) const;

	/// Get the neighbors of a CompositeVertex in composite_map
	/// \param[in] vertex
	std::vector<CompositeVertex> getNeighbors(CompositeVertex v);

	/// A Star
	std::vector<CompositeVertex> AStar();

	std::pair <double,double> calculateKey(CompositeVertex u);

	template<class TF>
	void initLPA(TF &mPQ);

	bool update_predecessor(CompositeVertex u, CompositeVertex v, double uv_weight);

	template<class TF>
	void update_vertex(TF &mPQ, CompositeVertex u);

	template<class TF>
	std::vector<CompositeVertex> compute_shortest_path(TF &mPQ);

}; // class MINT

} // namespace MINT


namespace MINT
{

MINT::MINT(const ompl::base::SpaceInformationPtr &si)
	: ompl::base::Planner(si, "MINT")
	, mSpace(si->getStateSpace())
	, mRoadmapFileName("")
	, mConnectionRadius(1.0)
	, mCheckRadius(0.005*mConnectionRadius)
	, mTPG(si->getStateSpace())
{
	// Register my setting callbacks.
	Planner::declareParam<std::string>("roadmapFilename", this, &MINT::setRoadmapFileName, &MINT::getRoadmapFileName);
}

MINT::MINT(const ompl::base::SpaceInformationPtr &si,
	const std::string& roadmapFileName)
	: ompl::base::Planner(si, "MINT")
	, mSpace(si->getStateSpace())
	, mRoadmapFileName(roadmapFileName)
	, mConnectionRadius(0.04)
	, mCheckRadius(0.005*mConnectionRadius)
	, mTPG(si->getStateSpace())
{

	if (mRoadmapFileName == "")
		throw std::invalid_argument("Provide a non-empty path to roadmap.");
}

MINT::~MINT()
{
	// Do nothing.
}

// ===========================================================================================
void MINT::setup()
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

	create_vertices(left_graph,get(&VProp::state,left_graph),mRoadmapFileName,2,get(&EProp::prior,left_graph));
	create_edges(left_graph,get(&EProp::length,left_graph));

	VertexIter ind_vi, ind_vi_end;
	size_t i=0;
	for (boost::tie(ind_vi, ind_vi_end) = vertices(left_graph); ind_vi != ind_vi_end; ++ind_vi,++i)
	{
		put(&VProp::vertex_index,left_graph,*ind_vi,i);
	}

	create_vertices(right_graph,get(&VProp::state,right_graph),mRoadmapFileName,2,get(&EProp::prior,right_graph));
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
	//   graph[*vi].costToCome = std::numeric_limits<double>::infinity();
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
void MINT::clear()
{
	// Set default vertex values.
	CompositeVertexIter vi, vi_end;
	for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
	{
		graph[*vi].distance = std::numeric_limits<double>::infinity();
		graph[*vi].distanceLookahead = std::numeric_limits<double>::infinity();
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
void MINT::setProblemDefinition(const ompl::base::ProblemDefinitionPtr &pdef)
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

	if(!validityChecker->isValid(startState->state))
		throw ompl::Exception("Start configuration is in collision!");
	if(!validityChecker->isValid(goalState->state))
		throw ompl::Exception("Goal configuration is in collision!");

	// Add start and goal vertices to the graph
	mStartVertex = boost::add_vertex(graph);
	graph[mStartVertex].state = startState;
	graph[mStartVertex].vertex_index = boost::num_vertices(graph) - 1;

	mGoalVertex = boost::add_vertex(graph);
	graph[mGoalVertex].state = goalState;
	graph[mGoalVertex].vertex_index = boost::num_vertices(graph) - 1;

	// Assign default values
	// graph[mStartVertex].distance = 0;
	// graph[mStartVertex].heuristic = heuristicFunction(mStartVertex);
	// graph[mStartVertex].parent = -1;
	// graph[mStartVertex].visited = false;
	// graph[mStartVertex].status = CollisionStatus::FREE;

	// graph[mGoalVertex].costToCome = std::numeric_limits<double>::infinity();
	// graph[mGoalVertex].heuristic = 0;
	// graph[mGoalVertex].visited = false;
	// graph[mGoalVertex].status = CollisionStatus::FREE;


	double* startValues = startState->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  

	Eigen::VectorXd left_start_config(2);
	left_start_config << startValues[0], startValues[1];

	Eigen::VectorXd right_start_config(2);
	right_start_config << startValues[2], startValues[3];

	double* goalValues = goalState->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;  

	Eigen::VectorXd left_goal_config(2);
	left_start_config << goalValues[0], goalValues[1];

	Eigen::VectorXd right_goal_config(2);
	right_goal_config << goalValues[2], goalValues[3];

	// start and goal added to left graph 

	Vertex left_start_vertex;
	left_start_vertex = add_vertex(left_graph);
	left_graph[left_start_vertex].state = left_start_config;
	left_graph[left_start_vertex].vertex_index = boost::num_vertices(left_graph) - 1;

	Vertex left_goal_vertex;
	left_goal_vertex = add_vertex(left_graph);
	left_graph[left_goal_vertex].state = left_goal_config;
	left_graph[left_goal_vertex].vertex_index = boost::num_vertices(left_graph) - 1;

	VertexIter ind_vi, ind_vi_end;
	for (boost::tie(ind_vi, ind_vi_end) = vertices(left_graph); ind_vi != ind_vi_end; ++ind_vi)
	{
		double startDist = (left_graph[left_start_vertex].state-left_graph[*ind_vi].state).norm();
		double goalDist = (left_graph[left_goal_vertex].state-left_graph[*ind_vi].state).norm();

		if (startDist < mConnectionRadius)
		{
			if(left_start_vertex == *ind_vi)
				continue;
			std::pair<Edge,bool> newEdge = boost::add_edge(left_start_vertex, *ind_vi, left_graph);
			left_graph[newEdge.first].length = startDist;
			left_graph[newEdge.first].prior = 1.0;
			left_graph[newEdge.first].isEvaluated = false;
			left_graph[newEdge.first].status = CollisionStatus::FREE;
		}

		if (goalDist < mConnectionRadius)
		{
			if(left_goal_vertex == *ind_vi)
				continue;
			std::pair<Edge,bool> newEdge = boost::add_edge(left_goal_vertex, *ind_vi, left_graph);
			left_graph[newEdge.first].length = goalDist;
			left_graph[newEdge.first].prior = 1.0;
			left_graph[newEdge.first].isEvaluated = false;
			left_graph[newEdge.first].status = CollisionStatus::FREE;
		}
	}


	// start and goal added to right graph 

	Vertex right_start_vertex;
	right_start_vertex = add_vertex(right_graph);
	right_graph[right_start_vertex].state = right_start_config;
	right_graph[right_start_vertex].vertex_index = boost::num_vertices(right_graph) - 1;

	Vertex right_goal_vertex;
	right_goal_vertex = add_vertex(right_graph);
	right_graph[right_goal_vertex].state = right_goal_config;
	right_graph[right_goal_vertex].vertex_index = boost::num_vertices(right_graph) - 1;

	// VertexIter ind_vi, ind_vi_end;
	for (boost::tie(ind_vi, ind_vi_end) = vertices(right_graph); ind_vi != ind_vi_end; ++ind_vi)
	{
		double startDist = (right_graph[right_start_vertex].state-right_graph[*ind_vi].state).norm();
		double goalDist = (right_graph[right_goal_vertex].state-right_graph[*ind_vi].state).norm();

		if (startDist < mConnectionRadius)
		{
			if(right_start_vertex == *ind_vi)
				continue;
			std::pair<Edge,bool> newEdge = boost::add_edge(right_start_vertex, *ind_vi, right_graph);
			right_graph[newEdge.first].length = startDist;
			right_graph[newEdge.first].prior = 1.0;
			right_graph[newEdge.first].isEvaluated = false;
			right_graph[newEdge.first].status = CollisionStatus::FREE;
		}

		if (goalDist < mConnectionRadius)
		{
			if(right_goal_vertex == *ind_vi)
				continue;
			std::pair<Edge,bool> newEdge = boost::add_edge(right_goal_vertex, *ind_vi, right_graph);
			right_graph[newEdge.first].length = goalDist;
			right_graph[newEdge.first].prior = 1.0;
			right_graph[newEdge.first].isEvaluated = false;
			right_graph[newEdge.first].status = CollisionStatus::FREE;
		}
	}

	// Populate the tensor product generator class
	mTPG.populateMaps(left_graph,right_graph,graph,mStartVertex,mGoalVertex);
}

std::vector<CompositeVertex> MINT::getNeighbors(CompositeVertex v)
{
	std::vector <CompositeVertex> neighbors;
	Eigen::VectorXd goal_config(4);
	double* goal_values = graph[mGoalVertex].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
	goal_config << goal_values[0], goal_values[1], goal_values[2], goal_values[3];
	
	neighbors = mTPG.getNeighborsImplicitTPG(v, graph, left_graph, right_graph, goal_config);
	
	// Initialize neighbor values right now being set in BGL_Definitions
	return neighbors;

}

// std::vector<CompositeVertex> MINT::AStar()
// {
// 	// Set default vertex values.
// 	CompositeVertexIter vi, vi_end;
// 	for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
// 	{
// 	  graph[*vi].costToCome = std::numeric_limits<double>::infinity();
// 	  graph[*vi].heuristic = std::numeric_limits<double>::infinity();
// 	  graph[*vi].visited = false;
// 	  graph[*vi].status = CollisionStatus::FREE;
// 	}

// 	graph[mStartVertex].costToCome = 0;
// 	graph[mStartVertex].heuristic = heuristicFunction(mStartVertex);
// 	graph[mStartVertex].parent = -1;

// 	graph[mGoalVertex].heuristic = 0;

// 	// Priority Function: f-value
// 	auto cmpFValue = [&](const CompositeVertex& left, const CompositeVertex& right)
// 	{
// 		double estimateLeft = estimateTotalCost(left);
// 		double estimateRight = estimateTotalCost(right);

// 		if (estimateRight - estimateLeft > 0)
// 			return true;
// 		if (estimateLeft - estimateRight > 0)
// 			return false;
// 		if (left < right)
// 			return true;
// 		else
// 			return false;
// 	};

// 	std::set<CompositeVertex, decltype(cmpFValue)> qUseful(cmpFValue);

// 	bool solutionFound = false;

// 	graph[mStartVertex].visited = true;
// 	qUseful.insert(mStartVertex);

// 	while(qUseful.size()!=0)
// 	{
// 		CompositeVertex vTop = *qUseful.begin();
// 		qUseful.erase(qUseful.begin());
// 		if(vTop == mGoalVertex)
// 		{
// 			solutionFound = true;
// 			break;      
// 		}

// 		std::vector <CompositeVertex> neighbors = getNeighbors(vTop);

// 		for (auto ai = neighbors.begin() ; ai != neighbors.end(); ++ai) 
// 		{
// 			// displayGraph(graph);
// 			CompositeVertex successor = *ai;
// 			CompositeEdge uv = getEdge(successor,vTop);
// 				// std::cout<<"Edge is Free!"<<std::endl;
// 			double edgeLength = graph[uv].length;
// 			double new_cost = graph[vTop].costToCome + edgeLength;
// 			if(new_cost < graph[successor].costToCome)
// 			{
// 			 graph[successor].costToCome = new_cost;
// 			 qUseful.insert(successor);
// 			 graph[successor].parent= vTop;
// 			}
// 		}
// 	}
// 	if (graph[mGoalVertex].costToCome == std::numeric_limits<double>::infinity())
// 		return std::vector<CompositeVertex>();

// 	std::vector<CompositeVertex> path;
	
// 	CompositeVertex node = mGoalVertex;
	
// 	while(node!=mStartVertex)
// 	{
// 		path.push_back(node);
// 		node=graph[node].parent;
// 	}
// 	path.push_back(mStartVertex);
// 	std::reverse(path.begin(), path.end());
// 	return path;
// }

// ===========================================================================================
ompl::base::PlannerStatus MINT::solve(const ompl::base::PlannerTerminationCondition & ptc)
{
	auto cmpKeyValue = [&](const CompositeVertex& left, const CompositeVertex& right)
	{
		std::pair<double,double> estimateLeft = calculateKey(left);
		std::pair<double,double> estimateRight = calculateKey(right);

		if (estimateRight.first - estimateLeft.first > eps)
			return true;
		if (estimateLeft.first - estimateRight.first > eps)
			return false;
		if (estimateRight.second - estimateLeft.second > eps)
			return true;
		if (estimateLeft.second - estimateRight.second > eps)
			return false;
		if (left < right)
			return true;
		else
			return false;
	};

	/// Priority Queue of LPAStar
	std::set<CompositeVertex, decltype(cmpKeyValue)> mPQ(cmpKeyValue);

	initLPA(mPQ);
	bool solutionFound = false;

	// Log Time
	std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};

	size_t numSearches = 0;

	while(!solutionFound)
	{
		std::cout<<"Search: "<<++numSearches<<std::endl;
		std::vector<CompositeVertex> shortestPath = compute_shortest_path(mPQ);
		// displayPath(graph,shortestPath);
		if(shortestPath.size()==0)
			break;

		std::cout<<"PATH: ";
		for(CompositeVertex &nodes: shortestPath )
			std::cout<<graph[nodes].vertex_index<<" ";
		std::cout<<std::endl;

		solutionFound = true;
		for(size_t i=0; i<shortestPath.size()-1; i++)
		{
			CompositeEdge uv = getEdge(shortestPath.at(i),shortestPath.at(i+1));            
			if(!graph[uv].isEvaluated)
			{
				graph[uv].isEvaluated = true;
				if(!evaluateEdge(uv))
				{
					graph[uv].length = std::numeric_limits<double>::infinity();
					if (update_predecessor(source(uv,graph),target(uv,graph),graph[uv].length))
						update_vertex(mPQ,target(uv,graph));

					// Update reverse direction.
					if (update_predecessor(target(uv,graph),source(uv,graph),graph[uv].length))
						update_vertex(mPQ,source(uv,graph));

					solutionFound = false;
					break;
				}
			}
		}
	}
	// std::vector<CompositeVertex> shortestPath = AStar();
	// if(shortestPath.size()==0)
	//  OMPL_INFORM("Solution NOT PRESENT!");

	std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
	std::chrono::duration<double> elapsedSeconds{endTime-startTime};
	mSearchTime = elapsedSeconds.count() - mEdgeEvaluationsTime - mLogTime;

	if(solutionFound)
	{
		OMPL_INFORM("Solution Found!");
		mBestPathCost = graph[mGoalVertex].distance;
		pdef_->addSolutionPath(constructSolution(mStartVertex, mGoalVertex));

		OMPL_INFORM("Number of Edges Rewired:     %d", mNumEdgeRewires);
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

void MINT::setRoadmapFileName(const std::string& roadmapFileName)
{
	mRoadmapFileName = roadmapFileName;
}

void MINT::setConnectionRadius(double connectionRadius)
{
	mConnectionRadius = connectionRadius;
}

void MINT::setCheckRadius(double checkRadius)
{
	mCheckRadius = checkRadius;
}
// ===========================================================================================
// Getters

std::string MINT::getRoadmapFileName() const
{
	return mRoadmapFileName;
}

double MINT::getConnectionRadius() const
{
	return mConnectionRadius;
}

double MINT::getCheckRadius() const
{
	return mCheckRadius;
}

CompositeVertex MINT::getStartVertex() const
{
	return mStartVertex;
}

CompositeVertex MINT::getGoalVertex() const
{
	return mGoalVertex;
}

double MINT::getBestPathCost() const
{
	return mBestPathCost;
}

std::size_t MINT::getNumEdgeEvaluations() const
{
	return mNumEdgeEvals;
}

std::size_t MINT::getNumEdgeRewires() const
{
	return mNumEdgeRewires;
}

double MINT::getEdgeEvaluationsTime() const
{
	return mEdgeEvaluationsTime;
}

double MINT::getSearchTime() const
{
	return mSearchTime;
}

// ===========================================================================================
ompl::base::PathPtr MINT::constructSolution(const CompositeVertex &start, const CompositeVertex &goal) const
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
void MINT::initializeEdgePoints(const CompositeEdge& e)
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
bool MINT::evaluateEdge(const CompositeEdge& e)
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
	
	// Evaluate Start and End States [we only assume states in self-collision are pruned out]
	if (checkResult && !validityChecker->isValid(startState))
	{
		graph[startVertex].status = CollisionStatus::BLOCKED;
		graph[e].status = CollisionStatus::BLOCKED;
		graph[e].length = std::numeric_limits<double>::max();
		checkResult = false;
	}

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
CompositeEdge MINT::getEdge(CompositeVertex u, CompositeVertex v) const
{
	CompositeEdge uv;
	bool edgeExists;
	boost::tie(uv, edgeExists) = edge(u, v, graph);

	return uv;
}

double MINT::heuristicFunction(CompositeVertex v) const
{
	return mSpace->distance(graph[v].state->state, graph[mGoalVertex].state->state);
	// double* config_values = 
	//  graph[v].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

	// Eigen::VectorXd composite_config(4);
	// composite_config << config_values[0],config_values[1],config_values[2],config_values[3];

	// double* goal_config_values = 
	//  graph[mGoalVertex].state->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

	// Eigen::VectorXd goal_composite_config(4);
	// composite_config << goal_config_values[0],goal_config_values[1],
	//                      goal_config_values[2],goal_config_values[3];
	// return std::max((composite_config.segment(0,2)-goal_composite_config.segment(0,2)).norm(),
	//                      (composite_config.segment(2,2)-goal_composite_config.segment(2,2)).norm());
}


///////////////////////////LPAStar Functions////////////////////////////////////////

std::pair <double,double> MINT::calculateKey(CompositeVertex u)
{
  double minval = std::min(graph[u].distance, graph[u].distanceLookahead);
  return std::make_pair(minval+heuristicFunction(u), minval);
}

template<class TF>
void MINT::initLPA(TF &mPQ)
{
	CompositeVertexIter vi, vi_end;
	for (boost::tie(vi,vi_end)=vertices(graph); vi!=vi_end; ++vi)
	{
		graph[*vi].distanceLookahead = std::numeric_limits<double>::infinity();
		graph[*vi].distance = std::numeric_limits<double>::infinity();
	}
	graph[mStartVertex].parent = -1;
	graph[mStartVertex].distanceLookahead = 0;
	mPQ.clear();
	mPQ.insert(mStartVertex);
}

bool MINT::update_predecessor(CompositeVertex u, CompositeVertex v, double uv_weight)
{
	if (v == mStartVertex)
		return false;

	CompositeVertex v_pred = graph[v].parent;
	double v_look_old = graph[v].distanceLookahead;
	double v_look_u = graph[u].distance + uv_weight;

	if (v_pred == u) 
	{
		if (v_look_u == v_look_old)
		{
			return false;
		}
		else if (v_look_u < v_look_old)
		{
			graph[v].distanceLookahead=v_look_u;
			return true;
		}
		else 
		{
			double v_look_best = std::numeric_limits<double>::infinity();
			CompositeVertex v_pred_best;
			CompositeNeighborIter ai, ai_end;
			for (boost::tie(ai,ai_end)=adjacent_vertices(v,graph); ai!=ai_end; ai++)
			{
				double v_look_uu = graph[*ai].distance + graph[getEdge(*ai,v)].length;
				if (v_look_uu < v_look_best)
				{
					v_look_best = v_look_uu;
					v_pred_best = *ai;
				}
			}
			if (v_look_best != std::numeric_limits<double>::infinity())
				graph[v].parent = v_pred_best;
			if (v_look_best == v_look_old)
			{
				return false;
			}
			else
			{
				graph[v].distanceLookahead = v_look_best;
				return true;
			}
		}
	}
	else
	{
		if (v_look_u < v_look_old)
		{
			graph[v].parent = u;
			graph[v].distanceLookahead = v_look_u;
			return true;
		}
		else
		{
			return false;
		}
	}
}

template<class TF>
void MINT::update_vertex(TF &mPQ, CompositeVertex u)
{
	double u_dist = graph[u].distance;
	if(u_dist == graph[u].distanceLookahead)
	{
		if (mPQ.find(u)!=mPQ.end())
			mPQ.erase(u);
	}
	else // not consistent
	{
		if (mPQ.find(u)!=mPQ.end())
		{
			mPQ.erase(u);
			mPQ.insert(u);
		}
		else
			mPQ.insert(u);
	}
}
// void MINT::update_vertex(TF &mPQ, CompositeVertex node) 
// {
// 	if (node != mStartVertex) 
// 	{
// 		std::vector<double> lookaheadValues;
// 		CompositeNeighborIter ai, ai_end;
// 		for (boost::tie(ai,ai_end)=adjacent_vertices(node,graph); ai!=ai_end; ai++)
// 		{
// 			double curLookahead = graph[*ai].distance + graph[boost::edge(*ai, node, graph).first].length;
// 			lookaheadValues.push_back(curLookahead);
// 		}
// 		if (lookaheadValues.size() == 0) 
// 		{
// 			graph[node].distanceLookahead = std::numeric_limits<double>::infinity();
// 		} 
// 		else 
// 		{
// 			std::vector<double>::iterator it = std::min_element(
// 					std::begin(lookaheadValues), std::end(lookaheadValues));

// 			double minLookahead = *it;
// 			graph[node].distanceLookahead = minLookahead;
// 		}
// 	}

// 	if (mPQ.find(node) != mPQ.end())
// 		mPQ.erase(node);
	
// 	if (graph[node].distance != graph[node].distanceLookahead)
// 		mPQ.insert(node);
// }
template<class TF>
std::vector<CompositeVertex> MINT::compute_shortest_path(TF &mPQ)
{
	while (mPQ.size() && (calculateKey(*mPQ.begin()) < calculateKey(mGoalVertex) 
		|| graph[mGoalVertex].distanceLookahead != graph[mGoalVertex].distance) ) 
	{
		CompositeVertex u = *mPQ.begin();
		mPQ.erase(mPQ.begin());
		if (graph[u].distance > graph[u].distanceLookahead)
		{
			graph[u].distance = graph[u].distanceLookahead;
			std::vector<CompositeVertex> neighbors = getNeighbors(u);
			for(auto &successor: neighbors)
			{
				if (update_predecessor(u, successor, graph[getEdge(u,successor)].length))
				update_vertex(mPQ,successor);
			}
		}
		else
		{
			graph[u].distance = std::numeric_limits<double>::infinity();
			update_vertex(mPQ,u);
			std::vector<CompositeVertex> neighbors = getNeighbors(u);
			for(auto &successor: neighbors)
			{
				if (update_predecessor(u, successor, graph[getEdge(u,successor)].length))
					update_vertex(mPQ,successor);
			}
		}
	}
	if (graph[mGoalVertex].distance == std::numeric_limits<double>::infinity())
		return std::vector<CompositeVertex>();

	std::vector<CompositeVertex> path;	
	CompositeVertex node = mGoalVertex;
	
	while(node!=mStartVertex)
	{
		path.push_back(node);
		node=graph[node].parent;
		std::cout<<"loop";
	}
	path.push_back(mStartVertex);
	std::reverse(path.begin(), path.end());
	return path;
}


} // namespace MINT

#endif // MINT_MINT_HPP_

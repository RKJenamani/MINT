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
namespace MINT {

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

  /// The fixed roadmap over which the search is done.
  CompositeGraph graph;

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

  /// Returns g-value of vertex.
  /// \param[in] vertex CompositeVertex
  double estimateCostToCome(CompositeVertex vertex) const;

  /// Returns heursitic value of vertex.
  /// \param[in] vertex CompositeVertex
  double heuristicFunction(CompositeVertex vertex) const;

  /// Returns f-value of vertex.
  /// \param vertex CompositeVertex
  double estimateTotalCost(CompositeVertex vertex) const;

  /// Construct the solution.
  /// \param[in] start Start vertex
  /// \param[in] goal Goal vertex
  ompl::base::PathPtr constructSolution(const CompositeVertex& start, const CompositeVertex& goal) const;

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

  Graph individual_graph;

  create_vertices(individual_graph,get(&VProp::state,individual_graph),mRoadmapFileName,mSpace->getDimension(),get(&EProp::prior,individual_graph));
  create_edges(individual_graph,get(&EProp::length,individual_graph));

  VertexIter ind_vi, ind_vi_end;

  std::unordered_map<Eigen::VectorXd, CompositeVertex, matrix_hash<Eigen::VectorXd>> configToNodeMap;

  for(boost::tie(ind_vi,ind_vi_end) = vertices(individual_graph); ind_vi!=ind_vi_end; ind_vi++)
  {
    Eigen::VectorXd config = individual_graph[*ind_vi].state;
    utils::StateWrapperPtr newState(new utils::StateWrapper(mSpace));
    ompl::base::State *ver_state{newState->state};
    double *values{ver_state->as<ompl::base::RealVectorStateSpace::StateType>()->values};
    for (size_t ui = 0; ui < mSpace->getDimension(); ui++)
    {
      values[ui] = config[ui];
    }
    
    CompositeVertex newVertex = boost::add_vertex(graph);
    graph[newVertex].state = newState;
    configToNodeMap[config]=newVertex;
  }

  EdgeIter ind_ei, ind_ei_end;
  for (boost::tie(ind_ei, ind_ei_end) = edges(individual_graph); ind_ei != ind_ei_end; ++ind_ei)
  {
    CompositeVertex source_vertex = configToNodeMap[individual_graph[source(*ind_ei, individual_graph)].state];
    CompositeVertex target_vertex = configToNodeMap[individual_graph[target(*ind_ei, individual_graph)].state];
    std::pair<CompositeEdge,bool> newEdge = boost::add_edge(source_vertex, target_vertex, graph);
    graph[newEdge.first].length = mSpace->distance(graph[source_vertex].state->state, graph[target_vertex].state->state);
    graph[newEdge.first].prior = 1.0;
  }
  // Set default vertex values.
  CompositeVertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
  {
    graph[*vi].costToCome = std::numeric_limits<double>::infinity();
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

  mBestPathCost = std::numeric_limits<double>::infinity();
}

// ===========================================================================================
void MINT::clear()
{
  // Set default vertex values.
  CompositeVertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
  {
    graph[*vi].costToCome = std::numeric_limits<double>::infinity();
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

  mGoalVertex = boost::add_vertex(graph);
  graph[mGoalVertex].state = goalState;

  // Assign default values
  graph[mStartVertex].costToCome = 0;
  graph[mStartVertex].heuristic = heuristicFunction(mStartVertex);
  graph[mStartVertex].parent = -1;
  graph[mStartVertex].visited = false;
  graph[mStartVertex].status = CollisionStatus::FREE;

  graph[mGoalVertex].costToCome = std::numeric_limits<double>::infinity();
  graph[mGoalVertex].heuristic = 0;
  graph[mGoalVertex].visited = false;
  graph[mGoalVertex].status = CollisionStatus::FREE;

  CompositeVertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi)
  {
    double startDist = mSpace->distance(graph[*vi].state->state, startState->state);
    double goalDist  = mSpace->distance(graph[*vi].state->state, goalState->state);

    if (startDist < mConnectionRadius)
    {
      if(mStartVertex == *vi)
        continue;
      std::pair<CompositeEdge,bool> newEdge = boost::add_edge(mStartVertex, *vi, graph);
      graph[newEdge.first].length = startDist;
      graph[newEdge.first].prior = 1.0;
      graph[newEdge.first].isEvaluated = false;
      graph[newEdge.first].status = CollisionStatus::FREE;
    }

    if (goalDist < mConnectionRadius)
    {
      if(mGoalVertex == *vi)
        continue;
      std::pair<CompositeEdge,bool> newEdge = boost::add_edge(mGoalVertex, *vi, graph);
      graph[newEdge.first].length = goalDist;
      graph[newEdge.first].prior = 1.0;
      graph[newEdge.first].isEvaluated = false;
      graph[newEdge.first].status = CollisionStatus::FREE;
    }
  }
}

// ===========================================================================================
ompl::base::PlannerStatus MINT::solve(const ompl::base::PlannerTerminationCondition & ptc)
{
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

  std::set<CompositeVertex, decltype(cmpFValue)> qUseful(cmpFValue);

  bool solutionFound = false;

  // Log Time
  std::chrono::time_point<std::chrono::system_clock> startTime{std::chrono::system_clock::now()};

  graph[mStartVertex].visited = true;
  // qExtend.insert(mStartVertex);

  qUseful.insert(mStartVertex);

  // extendLazyBand(qExtend, qFrontier);

  std::vector<CompositeVertex> path;
  while(qUseful.size()!=0)
  {
    CompositeVertex vTop = *qUseful.begin();
    qUseful.erase(qUseful.begin());
    if(vTop == mGoalVertex)
    {
      OMPL_INFORM("Solution Found!");
      solutionFound = true;
      break;      
    }
    CompositeNeighborIter ai;
    CompositeNeighborIter aiEnd;
    for (tie(ai, aiEnd) = adjacent_vertices(vTop, graph); ai != aiEnd; ++ai) 
    {
      CompositeVertex successor = *ai;
      CompositeEdge uv = getEdge(successor,vTop);
      if(evaluateEdge(uv))
      {
        double edgeLength = graph[uv].length;
        double new_cost = graph[vTop].costToCome + edgeLength;
        if(new_cost < graph[successor].costToCome)
        {
         graph[successor].costToCome = new_cost;
         qUseful.insert(successor);
         graph[successor].parent= vTop;
        }
      }
    }
  }

  std::chrono::time_point<std::chrono::system_clock> endTime{std::chrono::system_clock::now()};
  std::chrono::duration<double> elapsedSeconds{endTime-startTime};
  mSearchTime = elapsedSeconds.count() - mEdgeEvaluationsTime - mLogTime;

  if(solutionFound)
  {
    mBestPathCost = estimateCostToCome(mGoalVertex);
    pdef_->addSolutionPath(constructSolution(mStartVertex, mGoalVertex));

    OMPL_INFORM("Number of Edges Rewired:     %d", mNumEdgeRewires);
    OMPL_INFORM("Number of Edges Evaluated:   %d", mNumEdgeEvals);
    OMPL_INFORM("Cost of goal:                %f", mBestPathCost);

    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }

  else
  {
    OMPL_INFORM("Solution NOT Found");
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

  unsigned int nStates = static_cast<unsigned int>(
                                            std::floor(graph[e].length / (2.0*mCheckRadius)));
  
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

// ===========================================================================================
double MINT::estimateCostToCome(CompositeVertex v) const
{
  // return graph[v].costToCome + graph[v].lazyCostToCome;
  return graph[v].costToCome;
}

double MINT::heuristicFunction(CompositeVertex v) const
{
  return mSpace->distance(graph[v].state->state, graph[mGoalVertex].state->state);
}

double MINT::estimateTotalCost(CompositeVertex v) const
{
  return estimateCostToCome(v) + heuristicFunction(v);
}

} // namespace MINT

#endif // MINT_MINT_HPP_

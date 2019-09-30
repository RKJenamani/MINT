#ifndef BGL_DEFINITIONS_
#define BGL_DEFINITIONS_

#include <vector>
#include <string>
#include <unordered_set>
#include <queue>
#include <exception>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/function.hpp>
#include <boost/graph/copy.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graphml.hpp>
#include <time.h>

#include <Eigen/Dense>
#include <iostream>
#include <boost/program_options.hpp>

#include <dart/dart.hpp>
#include "util.hpp"

namespace MINT {

namespace BGL_DEFINITIONS {

  enum CollisionStatus
  {
    BLOCKED,
    FREE
  };

/////////////////////// DEFINITIONS FOR COMPOSITE ROADMAP

  // Properties associated with each roadmap vertex.
  struct CompositeVProp
  {
    /// The underlying state of the vertex
    utils::StateWrapperPtr state;

    /// Cost-to-Come
    double costToCome;

    /// Heuristic value
    double heuristic;

    /// Parent
    std::size_t parent;

    /// Visited
    bool visited;

    /// Collision status
    CollisionStatus status;

  }; // struct CompositeVProp

  // Properties associated with each roadmap edge.
  struct CompositeEProp
  {
    /// The length of the edge using the space distance metric
    double length;

    /// Flag to check if edge is evaluated
    bool isEvaluated;

    /// States embedded in an edge
    std::vector<utils::StateWrapperPtr> edgeStates;

    /// Collision status
    CollisionStatus status;

    /// Prior over existence of edge
    double prior;

  }; // struct CompositeEProp

  // Helpful alias declarations
  /// Undirected Boost graph
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, CompositeVProp, CompositeEProp> CompositeGraph;

  /// Boost vertex
  typedef boost::graph_traits<CompositeGraph>::vertex_descriptor CompositeVertex;

  /// Boost vertex iterator
  typedef boost::graph_traits<CompositeGraph>::vertex_iterator CompositeVertexIter;

  /// Boost edge
  typedef boost::graph_traits<CompositeGraph>::edge_descriptor CompositeEdge;

  /// Boost edge iterator
  typedef boost::graph_traits<CompositeGraph>::edge_iterator CompositeEdgeIter;

  /// Boost graph neighbor iterator
  typedef boost::graph_traits<CompositeGraph>::adjacency_iterator CompositeNeighborIter;

  /// Map each vertex to a unique ID
  typedef boost::property_map<CompositeGraph, boost::vertex_index_t CompositeVProp::*>::type CompositeVertexIndexMap;

  /// Map each vertex to the underlying state [read from the graphml file]
  typedef boost::property_map<CompositeGraph, utils::StateWrapperPtr CompositeVProp::*>::type CompositeVPStateMap;

  /// Map each edge to a unique ID
  typedef boost::property_map<CompositeGraph, boost::edge_index_t CompositeEProp::*>::type CompositeEdgeIndexMap;

  /// Map each edge to its length
  typedef boost::property_map<CompositeGraph, double CompositeEProp::*>::type CompositeEPLengthMap;

  /// Map each edge to its existence prior
  typedef boost::property_map<CompositeGraph, double CompositeEProp::*>::type CompositeEPPriorMap;


///////////////////////// DEFINITIONS FOR INDIVIDUAL ROADMAP

  // Properties associated with each roadmap vertex.
  struct VProp
  {
    /// The underlying state of the vertex
    //State (configuration)
    Eigen::VectorXd state;

    /// Cost-to-Come
    double costToCome;

    /// Heuristic value
    double heuristic;

    /// Parent
    std::size_t parent;

    /// Visited
    bool visited;

    /// Collision status
    CollisionStatus status;

  }; // struct VProp

  // Properties associated with each roadmap edge.
  struct EProp
  {
    /// The length of the edge using the space distance metric
    double length;

    /// Flag to check if edge is evaluated
    bool isEvaluated;

    /// Collision status
    CollisionStatus status;

    /// Prior over existence of edge
    double prior;

  }; // struct EProp

  // Helpful alias declarations
  /// Undirected Boost graph
  typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VProp, EProp> Graph;

  /// Boost vertex
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;

  /// Boost vertex iterator
  typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;

  /// Boost edge
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;

  /// Boost edge iterator
  typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;

  /// Boost graph neighbor iterator
  typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

  /// Map each vertex to a unique ID
  typedef boost::property_map<Graph, boost::vertex_index_t VProp::*>::type VertexIndexMap;

  /// Map each vertex to the underlying state [read from the graphml file]
  typedef boost::property_map<Graph, utils::StateWrapperPtr VProp::*>::type VPStateMap;

  /// Map each edge to a unique ID
  typedef boost::property_map<Graph, boost::edge_index_t EProp::*>::type EdgeIndexMap;

  /// Map each edge to its length
  typedef boost::property_map<Graph, double EProp::*>::type EPLengthMap;

  /// Map each edge to its existence prior
  typedef boost::property_map<Graph, double EProp::*>::type EPPriorMap; 

} // namespace BGL_DEFINITIONS

} // namespace MINT

#endif
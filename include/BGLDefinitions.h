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
#include <time.h>

#include <Eigen/Dense>
#include <iostream>
#include <boost/program_options.hpp>

#include <dart/dart.hpp>


namespace BGL_DEFINITIONS
{
  struct VProp
  {
    //State (configuration)
    Eigen::VectorXd state;

    size_t vertex_index;

    double rhsValue;
    double gValue;
    //name
    std::string name;
  };

  struct EProp
  {
    // Weight of edge
    double weight;
    // need to make it compile :D
    std::string prior;

    bool evaluated = false;
  };

  // Graph type definitions
  typedef boost::adjacency_list< boost::hash_setS, boost::listS, boost::undirectedS, VProp, EProp> Graph;
  typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
  typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;
  typedef boost::graph_traits<Graph>::edge_descriptor Edge;
  typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;
  typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIter;
  typedef boost::graph_traits<Graph>::in_edge_iterator InEdgeIter;
  typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

  //Creating property maps for vertices
  typedef boost::property_map<Graph, std::string VProp::*>::type VPNameMap;
  typedef boost::property_map<Graph, Eigen::VectorXd VProp::*>::type VPStateMap;
  typedef boost::property_map<Graph, size_t VProp::*>::type VPIndexMap;
  typedef boost::property_map<Graph, double VProp::*>::type VPrhsMap;
  typedef boost::property_map<Graph, double VProp::*>::type VPgMap;

  //Creating property maps for edges
  typedef boost::property_map<Graph, double EProp::*>::type EPWeightMap;
  typedef boost::property_map<Graph, std::string EProp::*>::type EPPriorMap;
  typedef boost::property_map<Graph, bool EProp::*>::type EPEvaluatedMap;


} // namespace BGL_DEFINITIONS

#endif

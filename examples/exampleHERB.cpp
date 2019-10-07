// Standard C++ libraries
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <queue>

// Boost libraries
// #include <boost/shared_ptr.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/function.hpp>
#include <boost/program_options.hpp>

// OMPL base libraries
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Custom header files
#include "MINT/MINT.hpp"
#include "HERB/HERButil.hpp"

namespace po = boost::program_options;

bool isPointValid(HERBUtil &collision_space, const ompl::base::State *state)
{
  // Obtain the state values
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  Eigen::VectorXd config(14);
  
  // Get the required point on the map
  config << values[0],values[1],values[2],values[3],values[4],values[5],values[6], //CHANGE LATER!
    values[7],values[8],values[9],values[10],values[11],values[12],values[13];
  
  return collision_space.getCollisionStatusVertex(config);
}

void executePath(std::string obstacleFile, HERBUtil &collision_space,
                 std::shared_ptr<ompl::geometric::PathGeometric> path)
{
  // Get state count
  int pathSize = path->getStateCount();

  std::vector<Eigen::VectorXd> configs; 

  for (int i = 0; i < pathSize; ++i)
  {
    auto uState = path->getState(i);
    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    Eigen::VectorXd config(14);
    config << u[0],u[1],u[2],u[3],u[4],u[5],u[6],u[7],u[8],u[9],u[10],u[11],u[12],u[13]; //TO CHANGE!!

    configs.push_back(config);
  }

  collision_space.executeTogether(configs);
  std::cout<<std::endl<<"Input [ENTER] to exit: ";
  std::cin.get();
}

ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
make_state(const ompl::base::StateSpacePtr space, Eigen::VectorXd config)
{
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace>state(space);
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  for (size_t ui = 0; ui < Space->getDimension(); ui++)
  {
    values[ui] = config[ui];
  }
  return state;
}

int main(int argc, char *argv[])
{
  po::options_description desc("HERB Planner Options");
  desc.add_options()
      ("help,h", "produce help message")
      ("left_graph,g", po::value<std::string>()->default_value(""), "Path to Left Graph")
      ("right_graph,g", po::value<std::string>()->default_value(""), "Path to Right Graph")
      ("source,s", po::value<std::vector<float> >()->multitoken(), "source configuration")
      ("target,t", po::value<std::vector<float> >()->multitoken(), "target configuration")
      ("execute,d", po::bool_switch()->default_value(true), "Enable to execute final path")
  ;

  // Read arguments
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
      std::cout << desc << std::endl;
      return 1;
  }

  std::string left_graph_file(vm["left_graph"].as<std::string>());
  std::string right_graph_file(vm["right_graph"].as<std::string>());
  if (left_graph_file == "")
    left_graph_file = "/home/rajat/personalrobotics/bgl_minimal_example/data/halton_2d_withedges.graphml";
  if (right_graph_file == "")
    right_graph_file = "/home/rajat/personalrobotics/bgl_minimal_example/data/halton_2d_withedges.graphml";
  std::string obstacle_file(vm["obstaclefile"].as<std::string>());
  if (obstacle_file == "")
    obstacle_file = "/home/rajat/personalrobotics/ompl_ws/src/MINT/data/obstacles/OneWall2D.png";
  std::vector<float> source(vm["source"].as<std::vector< float> >());
  std::vector<float> target(vm["target"].as<std::vector< float> >());
  bool execute(vm["execute"].as<bool>());

  // Define the state space: R^2
  std::shared_ptr<ompl::base::RealVectorStateSpace> space(
                                                      new ompl::base::RealVectorStateSpace(14));
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // Space Information
  HERBUtil collision_space;
  std::function<bool(const ompl::base::State*)> isStateValid = std::bind(
                                                    isPointValid, collision_space, std::placeholders::_1);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
    
  Eigen::VectorXd source_config(space->getDimension());
  source_config << source[0],source[1],source[2],source[3],source[4],source[5],source[6],
    source[7],source[8],source[9],source[10],source[11],source[12],source[13]; //TO CHANGE!!

  Eigen::VectorXd target_config(space->getDimension());
  target_config << target[0],target[1],target[2],target[3],target[4],target[5],target[6],
    target[7],target[8],target[9],target[10],target[11],target[12],target[13]; //TO CHANGE!!
  ;
  pdef->addStartState(make_state(space, source_config));
  pdef->setGoalState(make_state(space, target_config));

  // Setup planner
  MINT::MINT planner(si, left_graph_file);

  planner.setup();
  planner.setProblemDefinition(pdef);

  // Solve the motion planning problem
  ompl::base::PlannerStatus status;
  status = planner.solve(ompl::base::plannerNonTerminatingCondition());

  if (execute)
  {
    // Execute path and specify path size
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
    {
      auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
                                                              pdef->getSolutionPath());
      executePath(obstacle_file, collision_space, path);
    }
  }
  else
  {
    // Get planner data if required
    std::cout << "Exiting Cleanly" << std::endl;
  }

  return 0;
}

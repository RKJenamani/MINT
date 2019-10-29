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

namespace po = boost::program_options;

/// Check if the point is within defined hyperrectangle
/// This is bound to the stateValidityChecker of the ompl StateSpace
/// \param[in] image Obstacle image (grayscale)
/// \param[in] state The ompl state to check for validity
/// \return True if the state is collision-free
bool isPointValid(cv::Mat image, const ompl::base::State *state)
{
  // Obtain the state values
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  // Get the required point on the map
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  //left agent
  double left_x_point = values[0]*numberOfColumns;
  double left_y_point = (1 - values[1])*numberOfRows;
  cv::Point left_point((int)left_x_point, (int)left_y_point);

  // Collision Check for left agent with environment
  int left_intensity = (int)image.at<uchar>(left_point.y, left_point.x);
  if (left_intensity == 0) // Pixel is black
    return false;

  //right agent
  double right_x_point = values[2]*numberOfColumns;
  double right_y_point = (1 - values[3])*numberOfRows;
  cv::Point right_point((int)right_x_point, (int)right_y_point);

  // Collision Check for right agent with environment
  int right_intensity = (int)image.at<uchar>(right_point.y, right_point.x);
  if (right_intensity == 0) // Pixel is black
    return false;

  // Collision check of left-agent and right-agent with each other
  if((right_point.x-left_point.x)*(right_point.x-left_point.x) + 
    (right_point.y-left_point.y)*(right_point.y-left_point.y) < 9 )
    return false;

  return true;
}

/// Displays path
/// \param[in] obstacleFile The file with obstacles stored
/// \param[in] path OMPL path
void displayPath(std::string obstacleFile,
                 std::shared_ptr<ompl::geometric::PathGeometric> path)
{
  // Get state count
  int pathSize = path->getStateCount();

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  for (int i = 0; i < pathSize - 1; ++i)
  {
    auto uState = path->getState(i);
    auto vState = path->getState(i+1);
    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

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

/// Makes an OMPL state out of given x and y coordinates
/// \param[in] space State space
/// \param[in] x X-Coordinate
/// \param[in] y Y-Coordinate
/// \return OMPL state corresponding to (x, y)
ompl::base::ScopedState<ompl::base::RealVectorStateSpace>
make_state(const ompl::base::StateSpacePtr space, double left_x, double left_y, double right_x, double right_y)
{
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace>state(space);
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  values[0] = left_x;
  values[1] = left_y;
  values[2] = right_x;
  values[3] = right_y;
  return state;
}

int main(int argc, char *argv[])
{
  po::options_description desc("2D Map Planner Options");
  desc.add_options()
      ("help,h", "produce help message")
      ("left_graph,g", po::value<std::string>()->default_value(""), "Path to Left Graph")
      ("right_graph,g", po::value<std::string>()->default_value(""), "Path to Right Graph")
      ("obstaclefile,o", po::value<std::string>()->default_value(""), "Path to Obstacles File")
      ("source,s", po::value<std::vector<float> >()->multitoken(), "source configuration")
      ("target,t", po::value<std::vector<float> >()->multitoken(), "target configuration")
      ("display,d", po::bool_switch()->default_value(false), "Enable to display final path")
      ("LPAStar", po::bool_switch()->default_value(false), "Enable to use LPAStar as planner")
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
    left_graph_file = "/home/rajat/melodic_ws/src/MINT/data/graphs/halton_2d_withedges.graphml";
  if (right_graph_file == "")
    right_graph_file = "/home/rajat/melodic_ws/src/MINT/data/graphs/halton_2d_withedges.graphml";
  std::string obstacle_file(vm["obstaclefile"].as<std::string>());
  if (obstacle_file == "")
    obstacle_file = "/home/rajat/melodic_ws/src/MINT/data/obstacles/circle2D.png";
  std::vector<float> source(vm["source"].as<std::vector< float> >());
  std::vector<float> target(vm["target"].as<std::vector< float> >());
  bool display(vm["display"].as<bool>());
  bool useLPAStar(vm["LPAStar"].as<bool>());

  // Define the state space: R^2
  std::shared_ptr<ompl::base::RealVectorStateSpace> space(
                                                      new ompl::base::RealVectorStateSpace(4));
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // Space Information
  cv::Mat image = cv::imread(obstacle_file, 0);
  std::function<bool(const ompl::base::State*)> isStateValid = std::bind(
                                                    isPointValid, image, std::placeholders::_1);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->addStartState(make_state(space, source[0], source[1], source[2], source[3]));
  pdef->setGoalState(make_state(space, target[0], target[1], target[2], target[3]));

  if(useLPAStar)
  {
    // Setup planner
    MINT::MINT planner(std::string("LPAStar"), si, left_graph_file,right_graph_file);

    planner.setup();
    planner.setProblemDefinition(pdef);

    std::cout<<"CALLING SOLVE!"<<std::endl;
    // Solve the motion planning problem
    ompl::base::PlannerStatus status;
    status = planner.solve(ompl::base::plannerNonTerminatingCondition());

    if (display)
    {
      // Display path and specify path size
      if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
      {
        auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
                                                                pdef->getSolutionPath());
        displayPath(obstacle_file, path);
      }
    }
    else
    {
      // Get planner data if required
      std::cout << "Exiting Cleanly" << std::endl;
    }
  }
  else
  {
    // Setup planner
    MINT::MINT planner(std::string("LazySP"), si, left_graph_file,right_graph_file);

    planner.setup();
    planner.setProblemDefinition(pdef);

    std::cout<<"CALLING SOLVE!"<<std::endl;
    // Solve the motion planning problem
    ompl::base::PlannerStatus status;
    status = planner.solve(ompl::base::plannerNonTerminatingCondition());

    if (display)
    {
      // Display path and specify path size
      if (status == ompl::base::PlannerStatus::EXACT_SOLUTION)
      {
        auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(
                                                                pdef->getSolutionPath());
        displayPath(obstacle_file, path);
      }
    }
    else
    {
      // Get planner data if required
      std::cout << "Exiting Cleanly" << std::endl;
    }
  }

  return 0;
}
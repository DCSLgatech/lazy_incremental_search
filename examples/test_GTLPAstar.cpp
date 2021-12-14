// Standard C++ libraries
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <string>

// Boost libraries
#include <boost/function.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/program_options.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/shared_ptr.hpp>

// OMPL base libraries
#include <ompl/base/Planner.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// roscpp
#include "ros/ros.h"

// Custom header files
#include "lgls/GTLPAstar.hpp"

namespace po = boost::program_options;

/// Check if the point is within defined hyperrectangle
/// This is bound to the stateValidityChecker of the ompl StateSpace
/// \param[in] image Obstacle image (grayscale)
/// \param[in] state The ompl state to check for validity
/// \return True if the state is collision-free
bool isPointValid(cv::Mat image, const ompl::base::State* state) {
  // Obtain the state values
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  // Get the required point on the map
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;
  double x_point = values[0] * numberOfColumns;
  double y_point = (1 - values[1]) * numberOfRows;
  cv::Point point((int)x_point, (int)y_point);

  // Collision Check
  int intensity = (int)image.at<uchar>(point.y, point.x);
  if (intensity == 0) // Pixel is black
    return false;

  return true;
}

/// This highlights a subset of edges on top of the tree.
/// \param[in] obstacleFile The file with obstacles stored
/// \param[in] graph for visualizaiton
/// \param[in] vector of edges for highlight
void displayChanges(std::string obstacleFile, lgls::datastructures::Graph graph, std::vector<lgls::datastructures::Edge>& changed_edges) {
  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;


  // ====================== Draw Tree ==========================
  lgls::datastructures::EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {

    // Get End vertices
    lgls::datastructures::Vertex startVertex = source(*ei, graph);
    lgls::datastructures::Vertex endVertex = target(*ei, graph);

    auto uState = graph[startVertex].getState()->getOMPLState();
    auto vState = graph[endVertex].getState()->getOMPLState();

    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));
    cv::Point vPoint((int)(v[0] * numberOfColumns), (int)((1 - v[1]) * numberOfRows));

    if (graph[*ei].getColorStatus() == lgls::datastructures::ColorStatus::Unknown)
    {
      if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
      {
        cv::line(image, uPoint, vPoint, cv::Scalar(11, 58, 29), 0.2);
      }
    } else if (graph[*ei].getColorStatus() == lgls::datastructures::ColorStatus::Feasible)
    {
      if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
      {
        cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 2);
      }
      else {
        cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 1);
      }
    }
    else
    {
      if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
      {
        cv::line(image, uPoint, vPoint, cv::Scalar(0, 0, 255), 2);
      }
      else {
        cv::line(image, uPoint, vPoint, cv::Scalar(0, 0, 255), 1);
      }
    }
    // Draw Vertex
     cv::circle( image, uPoint,3,cv::Scalar( 100, 100, 0 ),cv::FILLED,cv::LINE_8 );
  }

  // ====================== Draw Changes ==========================
  for (std::vector<lgls::datastructures::Edge>::iterator ei = changed_edges.begin() ; ei != changed_edges.end(); ++ei)
  {
    lgls::datastructures::Vertex startVertex = source(*ei, graph);
    lgls::datastructures::Vertex endVertex = target(*ei, graph);

    auto uState = graph[startVertex].getState()->getOMPLState();
    auto vState = graph[endVertex].getState()->getOMPLState();

    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));
    cv::Point vPoint((int)(v[0] * numberOfColumns), (int)((1 - v[1]) * numberOfRows));


    if (graph[*ei].getColorStatus() == lgls::datastructures::ColorStatus::Unknown)
    {
        cv::line(image, uPoint, vPoint, cv::Scalar(11, 58, 29), 4);
    } else if (graph[*ei].getColorStatus() == lgls::datastructures::ColorStatus::Feasible)
    {
        cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 4);
    }
    else
    {
        cv::line(image, uPoint, vPoint, cv::Scalar(0, 0, 255), 4);
    }
  }
  cv::imwrite("imgs/changes.jpg", image);
}


/// Displays Tree
/// \param[in] obstacleFile The file with obstacles stored
/// \param[in] graph for visualizaiton
void displayTree(std::string obstacleFile, std::string prefix, lgls::datastructures::Graph graph) {
  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  lgls::datastructures::EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {

    // Get End vertices
    lgls::datastructures::Vertex startVertex = source(*ei, graph);
    lgls::datastructures::Vertex endVertex = target(*ei, graph);

    auto uState = graph[startVertex].getState()->getOMPLState();
    auto vState = graph[endVertex].getState()->getOMPLState();

    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));
    cv::Point vPoint((int)(v[0] * numberOfColumns), (int)((1 - v[1]) * numberOfRows));

    if (graph[*ei].getColorStatus() == lgls::datastructures::ColorStatus::Unknown)
    {
      if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
      {
        cv::line(image, uPoint, vPoint, cv::Scalar(11, 58, 29), 0.2);
      }
    } 
    else if (graph[*ei].getColorStatus() == lgls::datastructures::ColorStatus::Feasible)
    {
      if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
      {
        cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 2);
      }
      else {
        cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 1);
      }
    }
    else
    {
      if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
      {
        cv::line(image, uPoint, vPoint, cv::Scalar(0, 0, 255), 2);
      }
      else {
        cv::line(image, uPoint, vPoint, cv::Scalar(0, 0, 255), 1);
      }
    }

    // Draw Vertex
     cv::circle( image, uPoint,3,cv::Scalar( 100, 100, 0 ),cv::FILLED,cv::LINE_8 );
  }

  static int serial = 0;
  std::stringstream ss;
  ss << "imgs/"
     << prefix
     << std::setfill('0') << std::setw(3)
     << serial++
     << ".jpg";
  cv::imwrite(ss.str(), image);
}



/// Displays path
/// \param[in] obstacleFile The file with obstacles stored
/// \param[in] path OMPL path
void displayPath(std::string obstacleFile, std::shared_ptr<ompl::geometric::PathGeometric> path) {
  // Get state count
  int pathSize = path->getStateCount();

  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

  for (int i = 0; i < pathSize - 1; ++i) {
    auto uState = path->getState(i);
    auto vState = path->getState(i + 1);
    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));
    cv::Point vPoint((int)(v[0] * numberOfColumns), (int)((1 - v[1]) * numberOfRows));

    cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 3);
  }

  cv::imshow("Solution Path", image);
  cv::waitKey(0);
}

/// Creates an OMPL state from state values.
/// \param[in] space The ompl space the robot is operating in.
/// \param[in] x The x-coordinate.
/// \param[in] y The y-coorindate.
ompl::base::ScopedState<ompl::base::RealVectorStateSpace> make_state(
    const ompl::base::StateSpacePtr space, double x, double y) {
  ompl::base::ScopedState<ompl::base::RealVectorStateSpace> state(space);
  double* values = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  values[0] = x;
  values[1] = y;
  return state;
}

/// The main function.
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "test2d_image");
  po::options_description desc("2D Map Planner Options");
  // clang-format off
  desc.add_options()
  ("help,h", "produce help message")
  ("source,s", po::value<std::vector<float> >()->multitoken(), "source configuration")
  ("target,t", po::value<std::vector<float> >()->multitoken(), "target configuration")
  ("graph,g", po::value<std::string>()->required(), "graph specification")
  ("obstacle1,o", po::value<std::string>()->required(), "obstacle image (for visualization)")
  ("obstacle2,k", po::value<std::string>()->required(), "obstacle image (for visualization)");
  // clang-format on

  // Read arguments
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  std::vector<float> source(vm["source"].as<std::vector<float> >());
  std::vector<float> target(vm["target"].as<std::vector<float> >());
  std::string graphLocation(vm["graph"].as<std::string>());
  std::string obstacleLocation1(vm["obstacle1"].as<std::string>());
  std::string obstacleLocation2(vm["obstacle2"].as<std::string>());

  // Define the state space: R^2
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(2);
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(0.0, 1.0);
  space->setLongestValidSegmentFraction(0.1 / space->getMaximumExtent());
  space->setup();

  // Space Information
  cv::Mat image = cv::imread(obstacleLocation1, 0);
  std::function<bool(const ompl::base::State*)> isStateValid
      = std::bind(isPointValid, image, std::placeholders::_1);
  ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));
  si->setStateValidityChecker(isStateValid);
  si->setup();

  // Problem Definition
  ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
  pdef->addStartState(make_state(space, source[0], source[1]));
  pdef->setGoalState(make_state(space, target[0], target[1]));

  // Setup planner
  lgls::GTLPAstar planner(si);
  planner.setConnectionRadius(0.04);
  planner.setCollisionCheckResolution(0.1);
  planner.setRoadmap(graphLocation);
  planner.setEpsilon(1.0);
  //planner.setDebugCallback(std::bind(displayTree, obstacleLocation1, "first", std::placeholders::_1));

  auto event = std::make_shared<lgls::event::ShortestPathEvent>();
  planner.setEvent(event);

  planner.setup();
  planner.setProblemDefinition(pdef);

  // Let planner evaluate all the vertices to identify changes next time.
  planner.perceiveNewWorld();
  // Solve the motion planning problem
  ompl::base::PlannerStatus status;
  status = planner.solve(ompl::base::plannerNonTerminatingCondition());

  // Obtain required data if plan was successful
  if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
    // Display path and specify path size
    auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
    std::cout << "Solution Path Cost: " << planner.getBestPathCost() << std::endl;
    std::cout << "Number of Edge Evaluations: " << planner.getNumberOfEdgeEvaluations()
              << std::endl;
    std::cout << "Number of Vertex Expansions: " << planner.getNumberOfVertexExpansions()
              << std::endl;
    // displayPath(obstacleLocation1, path);
    displayTree(obstacleLocation1, "first", planner.getGraph());
  }

  // NOW the map changes
  image = cv::imread(obstacleLocation2, 0);
  isStateValid  = std::bind(isPointValid, image, std::placeholders::_1);
  //planner.setDebugCallback(std::bind(displayTree, obstacleLocation2, "second", std::placeholders::_1));
  planner.getSpaceInformation()->setStateValidityChecker(isStateValid);
  planner.getSpaceInformation()->setup();

  std::cout << " World changed " << std::endl;

  if (planner.perceiveChanges()) {
    std::cout << "Yes we perceived the change" <<std::endl;

    std::vector<lgls::datastructures::Edge> changed_edges = planner.getPerceivedChangedEdges();

    displayChanges(obstacleLocation2, planner.getGraph(), changed_edges);

    //Replan
    status = planner.solve(ompl::base::plannerNonTerminatingCondition());

    // Obtain required data if plan was successful
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
      // Display path and specify path size
      auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
      std::cout << "Solution Path Cost: " << planner.getBestPathCost() << std::endl;
      std::cout << "Number of Edge Evaluations: " << planner.getNumberOfEdgeEvaluations()
                << std::endl;
      std::cout << "Number of Vertex Expansions: " << planner.getNumberOfVertexExpansions()
                << std::endl;
      displayTree(obstacleLocation2, "second", planner.getGraph());
    }
  }
  
  return 0;
}

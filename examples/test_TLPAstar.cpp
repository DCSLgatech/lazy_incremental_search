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
#include "lgls/TLPAstar.hpp"

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

// /// Displays Tree
// /// \param[in] obstacleFile The file with obstacles stored
// /// \param[in] graph for visualizaiton
// void displayTree(std::string obstacleFile, std::string prefix, lgls::datastructures::Graph graph) {
//   // Obtain the image matrix
//   cv::Mat image = cv::imread(obstacleFile, 1);
//   int numberOfRows = image.rows;
//   int numberOfColumns = image.cols;
//
//   lgls::datastructures::EdgeIter ei, ei_end;
//   for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei) {
//
//     // Get End vertices
//     lgls::datastructures::Vertex startVertex = source(*ei, graph);
//     lgls::datastructures::Vertex endVertex = target(*ei, graph);
//
//     auto uState = graph[startVertex].getState()->getOMPLState();
//     auto vState = graph[endVertex].getState()->getOMPLState();
//
//     double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
//     double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
//
//     cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));
//     cv::Point vPoint((int)(v[0] * numberOfColumns), (int)((1 - v[1]) * numberOfRows));
//
//     if (graph[*ei].getColorStatus() == lgls::datastructures::ColorStatus::Unknown)
//     {
//       if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
//       {
//         cv::line(image, uPoint, vPoint, cv::Scalar(11, 58, 29), 0.2);
//       }
//     }
//     else if (graph[*ei].getColorStatus() == lgls::datastructures::ColorStatus::Feasible)
//     {
//       if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
//       {
//         cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 2);
//       }
//       else {
//         cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 1);
//       }
//     }
//     else
//     {
//       if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
//       {
//         cv::line(image, uPoint, vPoint, cv::Scalar(0, 0, 255), 2);
//       }
//       else {
//         cv::line(image, uPoint, vPoint, cv::Scalar(0, 0, 255), 1);
//       }
//     }
//
//     // Draw Vertex
//      cv::circle( image, uPoint,3,cv::Scalar( 100, 100, 0 ),cv::FILLED,cv::LINE_8 );
//   }
//
//   cv::namedWindow( "TLPAstar", cv::WINDOW_AUTOSIZE );
//   cv::imshow("TLPAstar", image);
//   cv::waitKey(0);
// }
//
//
//
// /// Displays path
// /// \param[in] obstacleFile The file with obstacles stored
// /// \param[in] path OMPL path
// void displayPath(std::string obstacleFile, std::shared_ptr<ompl::geometric::PathGeometric> path) {
//   // Get state count
//   int pathSize = path->getStateCount();
//
//   // Obtain the image matrix
//   cv::Mat image = cv::imread(obstacleFile, 1);
//   int numberOfRows = image.rows;
//   int numberOfColumns = image.cols;
//
//   for (int i = 0; i < pathSize - 1; ++i) {
//     auto uState = path->getState(i);
//     auto vState = path->getState(i + 1);
//     double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
//     double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
//
//     cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));
//     cv::Point vPoint((int)(v[0] * numberOfColumns), (int)((1 - v[1]) * numberOfRows));
//
//     cv::line(image, uPoint, vPoint, cv::Scalar(255, 0, 0), 3);
//   }
//
//   cv::imshow("Solution Path", image);
//   cv::waitKey(0);
// }



void displayTreePath(std::string obstacleFile, int duration, lgls::datastructures::Graph graph, lgls::datastructures::Path path) {
  // Obtain the image matrix
  cv::Mat image = cv::imread(obstacleFile, 1);
  int numberOfRows = image.rows;
  int numberOfColumns = image.cols;

//=============================  Tree Drawing ================================//
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


    // Variables to draw a line
    int lineWidth;
    cv::Scalar lineColor;
    // Draw evaluated edges with color
    if (graph[*ei].getEvaluationStatus() == lgls::datastructures::EvaluationStatus::Evaluated)
    {
      // Get line Color, depending on collision or not
      if (graph[*ei].getCollisionStatus() == lgls::datastructures::CollisionStatus::Free)
      {
          // Blue
          lineColor = cv::Scalar(255,0,0);

      } // Okay, colliding edge
      else
      {
          // Red
          lineColor = cv::Scalar(0,0,255);
      }

      // Check if this belongs to tree
      if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
      {
        lineWidth = 3;
      }
      else
      {
        lineWidth = 1;
      }

      // Now actually draw the line cv::Scalar BGRA
      cv::line(image, uPoint, vPoint, lineColor, lineWidth);

    }   // Draw unevaluated edges with black light
    else // This edge is not evaluated
    {
        // thin edge
        if (graph[endVertex].getParent()==startVertex ||graph[startVertex].getParent()==endVertex  )
        {
          // black
          lineColor = cv::Scalar(0,0,0);
          lineWidth = 1;
          // Now actually draw the line cv::Scalar BGRA
          cv::line(image, uPoint, vPoint, lineColor, lineWidth);
        }

        // No else, no need to draw
    }

    // Draw Vertex
     cv::circle( image, uPoint,3,cv::Scalar( 100, 100, 0 ),cv::FILLED,cv::LINE_8 );
  } // for all the edges in the graph

//=============================  Path Drawing ================================//
  // Now Draw the solution path
  int solutionLineWidth  = 13;

  for (std::size_t i = path.size() - 1; i > 0; --i) {

    lgls::datastructures::Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i], path[i-1] , graph);

    if (!edgeExists) break;
    assert(edgeExists);

    if (graph[uv].getEvaluationStatus() == lgls::datastructures::EvaluationStatus::NotEvaluated)
    {
      // This path is not certified. don't draw
      break;
    }

    // Retreive the end points of child-parent edge
    auto uState = graph[path[i]].getState()->getOMPLState();
    auto vState = graph[path[i-1]].getState()->getOMPLState();

    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));
    cv::Point vPoint((int)(v[0] * numberOfColumns), (int)((1 - v[1]) * numberOfRows));

    cv::Scalar lineColor = cv::Scalar(255,0,0);;

    // Now actually draw the line cv::Scalar BGRA
    cv::line(image, uPoint, vPoint, lineColor, solutionLineWidth);

  } // End For path iteration


//============================  Acutal Rendering ==============================//
  // Draw strat and goal vertices ad hoc
  auto uState = graph[path[path.size()-1]].getState()->getOMPLState();
  auto vState = graph[path[0]].getState()->getOMPLState();

  double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));
  cv::Point vPoint((int)(v[0] * numberOfColumns), (int)((1 - v[1]) * numberOfRows));

  cv::circle( image, uPoint,15,cv::Scalar(0, 255, 0 ),cv::FILLED,cv::LINE_8 );
  cv::circle( image, vPoint,15,cv::Scalar(255, 0, 255 ),cv::FILLED,cv::LINE_8 );

  cv::namedWindow("BLGLS",cv::WINDOW_AUTOSIZE);
  cv::moveWindow("BLGLS",200,0);
  // cv::resizeWindow("Lazy COLPA*",200,200);
  cv::imshow("BLGLS", image);
  cv::waitKey(duration);

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
  lgls::TLPAstar planner(si);
  planner.setConnectionRadius(0.04);
  planner.setCollisionCheckResolution(0.1);
  planner.setRoadmap(graphLocation);
  planner.setTruncationFactor(1.44);


  planner.setup();
  planner.setProblemDefinition(pdef);

  // planner.generateNewSamples(400,false);

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
    // displayTree(obstacleLocation1, "first", planner.getGraph());
    displayTreePath(obstacleLocation1, 0 , planner.getGraph(), planner.getPath());
  }

  // NOW the map changes
  image = cv::imread(obstacleLocation2, 0);
  isStateValid  = std::bind(isPointValid, image, std::placeholders::_1);
  planner.getSpaceInformation()->setStateValidityChecker(isStateValid);
  planner.getSpaceInformation()->setup();

  std::cout << " World changed " << std::endl;

  if (planner.perceiveChanges()) {
    std::cout << "Yes we perceived the change" <<std::endl;

  }

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
    // displayTree(obstacleLocation2, "second", planner.getGraph());
    displayTreePath(obstacleLocation2, 0 , planner.getGraph(), planner.getPath());
  }

  return 0;
}

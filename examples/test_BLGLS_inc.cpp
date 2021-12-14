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
#include "lgls/BLGLS.hpp"

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


void displayTree(std::string obstacleFile, int duration, lgls::datastructures::Graph graph) {
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
    // cv::circle( image, uPoint,3,cv::Scalar( 100, 100, 0 ),cv::FILLED,cv::LINE_8 );
  } // for all the edges in the graph

//=============================  Path Drawing ================================//
  // Now Draw the solution path
  lgls::datastructures::Vertex pathVertex = 1;
  int solutionLineWidth  = 13;
  // Draw Path if all the edges along the path is evaluated
  while (graph[pathVertex].hasParent() && pathVertex !=0)
  {
    lgls::datastructures::Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(pathVertex,graph[pathVertex].getParent(), graph);

    if (!edgeExists) break;
    assert(edgeExists);

    if (graph[uv].getEvaluationStatus() == lgls::datastructures::EvaluationStatus::NotEvaluated)
    {
      // This path is not certified. don't draw
      break;
    }


    // Retreive the end points of child-parent edge
    auto uState = graph[pathVertex].getState()->getOMPLState();
    auto vState = graph[graph[pathVertex].getParent()].getState()->getOMPLState();

    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* v = vState->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));
    cv::Point vPoint((int)(v[0] * numberOfColumns), (int)((1 - v[1]) * numberOfRows));

    cv::Scalar lineColor = cv::Scalar(255,0,0);;

    // Now actually draw the line cv::Scalar BGRA
    cv::line(image, uPoint, vPoint, lineColor, solutionLineWidth);

    pathVertex = graph[pathVertex].getParent();

  } // end while path retrieve

//=============================  Vertex Drawing ================================//
  lgls::datastructures::VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(graph); vi != vi_end; ++vi) {

    auto uState = graph[*vi].getState()->getOMPLState();
    double* u = uState->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    cv::Point uPoint((int)(u[0] * numberOfColumns), (int)((1 - u[1]) * numberOfRows));

    // Draw Vertex
    cv::circle( image, uPoint,3,cv::Scalar( 100, 100, 0 ),cv::FILLED,cv::LINE_8 );

  } // End For vertex iteration


//============================  Start and Goal  ==============================//
  // Draw strat and goal vertices ad hoc
  auto uState = graph[0].getState()->getOMPLState();
  auto vState = graph[1].getState()->getOMPLState();

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


void updateSampleAvgVar(const int n, const double currentSample, double& previousAvg, double& previousVar){
  // First compute variance
  previousVar = (n-1)/n*previousVar + (currentSample-previousAvg)*(currentSample-previousAvg)/(n+1);
  previousAvg = (currentSample+(n-1)*(previousAvg))/n;
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


  // Experiment variables
  int totalRun = 100;
  int totalIteration = 10; // number of batch additions for each experiment
  int batchSize = 100; // number of samples added for each batch
  double inflationFactor = 1.0;
  double truncationFactor = 1.0;


  for (int lookahead = 1; lookahead < 7; lookahead++) {


    // Initiate run-wise vectors
    // Number of Edge Evaluation
    std::vector<std::vector<int>> runTotalNumEdgeEval;
    // Number of Vertex Expansion
    std::vector<std::vector<int>> runTotalNumVertexExp;
    // Total Edge Evaluation Time
    std::vector<std::vector<double>> runTotalEdgeEvalTime;
    // Total Vertex Exp Time
    std::vector<std::vector<double>> runTotalVertexExpTime;
    // Total Time spent
    std::vector<std::vector<double>> runTotalTimeSpent;
    // Solution
    std::vector<std::vector<double>> runSolutionLength;

    // Same planner setup,
    for (int runIter = 0; runIter < totalRun; runIter++)
    {

    //=================== Iteration wise variables  =======================//
      // Number of Edge Evaluation
      std::vector<int> iterTotalNumEdgeEval;
      // Number of Vertex Expansion
      std::vector<int> iterTotalNumVertexExp;
      // Total Edge Evaluation Time
      std::vector<double> iterTotalEdgeEvalTime;
      // Total Vertex Exp Time
      std::vector<double> iterTotalVertexExpTime;
      // Total Time spent
      std::vector<double> iterTotalTimeSpent;
      // Solution
      std::vector<double> iterSolutionLength;
      //========== Experiment results to be accumulated =====================//
      int totalNumberOfEdgeEvaluation = 0;
      int totalNumberOfVertexExpansion = 0;
      double totalEdgeEvaluationTime=0;
      double totalVertexExpansionTime=0;
      double totalTimeSpent = 0;
    //=================== planner setup =============================//
      // Problem Definition
      ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
      pdef->addStartState(make_state(space, source[0], source[1]));
      pdef->setGoalState(make_state(space, target[0], target[1]));

      // Setup planner
      lgls::BLGLS planner(si);
      planner.setConnectionRadius(0.1);
      planner.setCollisionCheckResolution(0.1);
      // planner.setInflationFactor(1);

      // auto event = std::make_shared<lgls::event::ShortestPathEvent>();
      auto event = std::make_shared<lgls::event::ConstantDepthEvent>(lookahead);
      planner.setEvent(event);

      planner.setup();
      planner.setProblemDefinition(pdef);

      planner.setDebugCallback(std::bind(displayTree, obstacleLocation1, 10, std::placeholders::_1));

      // Add Samples, make sure you add incrementally to match the connection radius
      planner.generateNewSamples(batchSize,false);
      planner.setInflationFactor(inflationFactor);
      planner.setTruncationFactor(truncationFactor);
    //=================== Main iteration loop  =============================//
      for (int i = 0; i < totalIteration; i++) {

        ROS_INFO("======= Lookahead %d: LGLS Planner Running %d - %d ==============",lookahead, runIter, i);
        // Solve the motion planning problem
        ompl::base::PlannerStatus status;
        status = planner.solve(ompl::base::plannerNonTerminatingCondition());

        // ==================== record the results ========================//
        totalNumberOfEdgeEvaluation = totalNumberOfEdgeEvaluation + planner.getNumberOfEdgeEvaluations();
        totalNumberOfVertexExpansion = totalNumberOfVertexExpansion + planner.getNumberOfVertexExpansions();

        if (planner.getNumberOfEdgeEvaluations()!=0)
        totalEdgeEvaluationTime = totalEdgeEvaluationTime
                + (planner.getNumberOfEdgeEvaluations()*planner.getAvgEdgeEvalTime() );

        if (planner.getNumberOfVertexExpansions()!=0)
        totalVertexExpansionTime = totalVertexExpansionTime
                + (planner.getNumberOfVertexExpansions()*planner.getAvgVertexExpTime() );

        totalTimeSpent = totalEdgeEvaluationTime+totalVertexExpansionTime;

        // Number of Edge Evaluation
        iterTotalNumEdgeEval.push_back(totalNumberOfEdgeEvaluation);
        // Number of Vertex Expansion
        iterTotalNumVertexExp.push_back(totalNumberOfVertexExpansion);
        // Total Edge Evaluation Time
        iterTotalEdgeEvalTime.push_back(totalEdgeEvaluationTime);
        // Total Vertex Exp Time
        iterTotalVertexExpTime.push_back(totalVertexExpansionTime);
        // Total Time spent
        iterTotalTimeSpent.push_back(totalTimeSpent);
        // Solution
        iterSolutionLength.push_back(planner.getBestPathCost());

        // Obtain required data if plan was successful
        if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
          // Display path and specify path size
          // auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
          std::cout << "Solution Path Cost: " << planner.getBestPathCost() << std::endl;
          std::cout << "Number of Edge Evaluations: " << planner.getNumberOfEdgeEvaluations()
                    << std::endl;
          std::cout << "Number of Vertex Expansions: " << planner.getNumberOfVertexExpansions()
                    << std::endl;

          // displayTree(obstacleLocation1, 1000, planner.getGraph());
          // planner.setInflationFactor(inflationFactor);
        }

        planner.generateNewSamples(batchSize,true);

      } // End for loop of iterations


      // Now push the results to run-wise vectors
      // Number of Edge Evaluation
      runTotalNumEdgeEval.push_back(iterTotalNumEdgeEval);
      // Number of Vertex Expansion
      runTotalNumVertexExp.push_back(iterTotalNumVertexExp);
      // Total Edge Evaluation Time
      runTotalEdgeEvalTime.push_back(iterTotalEdgeEvalTime);
      // Total Vertex Exp Time
      runTotalVertexExpTime.push_back(iterTotalVertexExpTime);
      // Total Time spent
      runTotalTimeSpent.push_back(iterTotalTimeSpent);
      // Solution
      runSolutionLength.push_back(iterSolutionLength);

    } // end for run iteration


    // ================== Statistical Work ===========================//
    // We will go over Runs and calculate avg and std for each iterations
    //              ________________
    //             |    |   |   |   |
    //        _____|____|_stat_ |   |
    //      /________________/| |   |
    //     /_______run______/ |_|___|
    //    /________________/  |
    //    |    |   |   |   |  |
    //    |    iteration   |  /
    //    |    |   |   |   | /
    //    |____|___|___|___|/
    //
    //
    //=================== Open up file streaming  =============================//
    // File streaming
    std::ofstream myfile;
    // std::string filetag = std::to_string(batchSize)+"_"+std::to_string(inflationFactor);

    std::string trimmedIF = std::to_string(inflationFactor).substr(0, std::to_string(inflationFactor).find(".") + 3);
    std::string trimmedTF = std::to_string(truncationFactor).substr(0, std::to_string(truncationFactor).find(".") + 3);
    std::string filetag = "l"+std::to_string(lookahead)+"-IF"+trimmedIF+"-TF"+trimmedTF+"-b"+std::to_string(batchSize);
    myfile.open("/home/jlim78/Desktop/BLGLSexperiments/2d/" + filetag + ".csv");

    myfile << "NumSamples, avgTotalNumEdgeEval, stdTotalNumEdgeEval," ;
    myfile << "avgTotalNumVertexExp, stdTotalNumVertexExp, ";
    myfile << "avgTotalEdgeEvalTime, stdTotalEdgeEvalTime,";
    myfile << "avgTotalVertexExpTime, stdTotalVertexExpTime,";
    myfile << "avgTotalTimeSpent, stdTotalTimeSpent,";
    myfile << "avgSolution, stdSolution\n";

    //=================== Main iteration loop  =============================//
    for (int i = 0; i < totalIteration; i++) {

      // Number of Edge Evaluation
      double avgTotalNumEdgeEval = 0;
      double varTotalNumEdgeEval = 0;
      // Number of Vertex Expansion
      double avgTotalNumVertexExp = 0;
      double varTotalNumVertexExp = 0;
      // Total Edge Evaluation Time
      double avgTotalEdgeEvalTime = 0;
      double varTotalEdgeEvalTime = 0;
      // Total Vertex Exp Time
      double avgTotalVertexExpTime = 0;
      double varTotalVertexExpTime = 0;
      // Total Time spent
      double avgTotalTimeSpent = 0;
      double varTotalTimeSpent = 0;
      // Solution
      double avgSolution = 0;
      double varSolution = 0;

      for (int runIter = 0; runIter < totalRun; runIter++)
      {
        int n = runIter+1;
        // Number of Edge Evaluation
        updateSampleAvgVar(n, runTotalNumEdgeEval.at(runIter).at(i), avgTotalNumEdgeEval, varTotalNumEdgeEval);
        // // Number of Vertex Expansion
        updateSampleAvgVar(n, runTotalNumVertexExp.at(runIter).at(i), avgTotalNumVertexExp, varTotalNumVertexExp);
        // // Total Edge Evaluation Time
        updateSampleAvgVar(n, runTotalEdgeEvalTime.at(runIter).at(i), avgTotalEdgeEvalTime, varTotalEdgeEvalTime);
        // // Total Vertex Exp Time
        updateSampleAvgVar(n, runTotalVertexExpTime.at(runIter).at(i), avgTotalVertexExpTime, varTotalVertexExpTime);
        // // Total Time spent
        updateSampleAvgVar(n, runTotalTimeSpent.at(runIter).at(i), avgTotalTimeSpent, varTotalTimeSpent);
        // // Solution
        updateSampleAvgVar(n, runSolutionLength.at(runIter).at(i), avgSolution, varSolution);

        // varTotalNumEdgeEval = calculateSampleVar(n, runTotalNumEdgeEval.at(runIter).at(i), avgTotalNumEdgeEval, varTotalNumEdgeEval);
        // avgTotalNumEdgeEval = calculateSampleAvg(n,runTotalNumEdgeEval.at(runIter).at(i),avgTotalNumEdgeEval);
        //
        // // Number of Vertex Expansion
        // varTotalNumVertexExp = calculateSampleVar(n, runTotalNumVertexExp.at(runIter).at(i), avgTotalNumVertexExp, varTotalNumVertexExp);
        // avgTotalNumVertexExp = calculateSampleAvg(n,runTotalNumVertexExp.at(runIter).at(i),avgTotalNumVertexExp);
        //
        // // Total Edge Evaluation Time
        // varTotalEdgeEvalTime = calculateSampleVar(n, runTotalEdgeEvalTime.at(runIter).at(i), avgTotalEdgeEvalTime, varTotalEdgeEvalTime);
        // avgTotalEdgeEvalTime = calculateSampleAvg(n,runTotalEdgeEvalTime.at(runIter).at(i),avgTotalEdgeEvalTime);
        //
        // // Total Vertex Exp Time
        // varTotalVertexExpTime = calculateSampleVar(n, runTotalVertexExpTime.at(runIter).at(i), avgTotalVertexExpTime, varTotalVertexExpTime);
        // avgTotalVertexExpTime = calculateSampleAvg(n,runTotalVertexExpTime.at(runIter).at(i),avgTotalVertexExpTime);
        //
        // // Total Time spent
        // varTotalTimeSpent = calculateSampleVar(n, runTotalTimeSpent.at(runIter).at(i), avgTotalTimeSpent, varTotalTimeSpent);
        // avgTotalTimeSpent = calculateSampleAvg(n,runTotalTimeSpent.at(runIter).at(i),avgTotalTimeSpent);
        //
        // // Solution
        // varSolution = calculateSampleVar(n, runSolutionLength.at(runIter).at(i), avgSolution, varSolution);
        // avgSolution = calculateSampleAvg(n,runSolutionLength.at(runIter).at(i),avgSolution);

      } // end for loop runwise

      // myfile << "NumSamples, avgTotalNumEdgeEval, stdTotalNumEdgeEval," ;
      // myfile << "avgTotalNumVertexExp, stdTotalNumVertexExp, ";
      // myfile << "avgTotalEdgeEvalTime, stdTotalEdgeEvalTime,";
      // myfile << "avgTotalVertexExpTime, stdTotalVertexExpTime,";
      // myfile << "avgTotalTimeSpent, stdTotalTimeSpent,";
      // myfile << "avgSolution, stdSolution\n";

      myfile <<  batchSize * (i+1) << ",";
      myfile <<  avgTotalNumEdgeEval << ",";
      myfile <<  std::sqrt(varTotalNumEdgeEval) << ",";

      myfile <<  avgTotalNumVertexExp << ",";
      myfile <<  std::sqrt(varTotalNumVertexExp) << ",";

      myfile <<  avgTotalEdgeEvalTime << ",";
      myfile <<  std::sqrt(varTotalEdgeEvalTime) << ",";

      myfile <<  avgTotalVertexExpTime << ",";
      myfile <<  std::sqrt(varTotalVertexExpTime) << ",";

      myfile <<  avgTotalTimeSpent << ",";
      myfile <<  std::sqrt(varTotalTimeSpent) << ",";

      myfile <<  avgSolution << ",";
      myfile <<  std::sqrt(varSolution) << "\n";
    } // end for iteration

    // Now close the file
    myfile.close();

  } // End of for loop lookahead





// // Main loop iterating over different lookahead steps
//   for (int lookahead = 1; lookahead < 11; lookahead++) {
//     //========== Experiment results to be recorded =======================//
//     int totalNumberOfEdgeEvaluation = 0;
//     int totalNumberOfVertexExpansion = 0;
//
//     double totalTimeSpent = 0;
//     double totalEdgeEvaluationTime=0;
//     double totalVertexExpansionTime=0;
//     //=================== planner setup =============================//
//     // Problem Definition
//     ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));
//     pdef->addStartState(make_state(space, source[0], source[1]));
//     pdef->setGoalState(make_state(space, target[0], target[1]));
//
//     // Setup planner
//     lgls::BLGLS planner(si);
//     planner.setConnectionRadius(0.1);
//     planner.setCollisionCheckResolution(0.1);
//     // planner.setInflationFactor(1);
//
//     // auto event = std::make_shared<lgls::event::ShortestPathEvent>();
//     auto event = std::make_shared<lgls::event::ConstantDepthEvent>(lookahead);
//     planner.setEvent(event);
//
//     planner.setup();
//     planner.setProblemDefinition(pdef);
//
//     planner.setDebugCallback(std::bind(displayTree, obstacleLocation1, 10, std::placeholders::_1));
//
//     // Add Samples, make sure you add incrementally to match the connection radius
//     planner.generateNewSamples(batchSize,false);
//     planner.setInflationFactor(inflationFactor);
//     planner.setTruncationFactor(truncationFactor);
//
//     //=================== open up file streaming  =============================//
//     // File streaming
//     std::ofstream myfile;
//     // std::string filetag = std::to_string(batchSize)+"_"+std::to_string(inflationFactor);
//
//     std::string trimmedIF = std::to_string(inflationFactor).substr(0, std::to_string(inflationFactor).find(".") + 3);
//     std::string trimmedTF = std::to_string(truncationFactor).substr(0, std::to_string(truncationFactor).find(".") + 3);
//     std::string filetag = "l"+std::to_string(lookahead)+"-IF"+trimmedIF+"-TF"+trimmedTF+"-b"+std::to_string(batchSize);
//     myfile.open("/home/jlim78/Desktop/BLGLSexperiments/2d/" + filetag + ".csv");
//
//     myfile << "numer of samples, numer of edge evaluations, number of vertex expansions," ;
//     myfile << "solution length, edge evaluation time, vertex expansion time,";
//     myfile << "total number of edges, total number of vertices,";
//     myfile << "total edge eval time, total vertex exp time, total time spent \n";
//
//     //=================== Main iteration loop  =============================//
//     for (int i = 1; i < totalIteration; i++) {
//
//       ROS_INFO("======= Lookahead %d: LGLS Planner Running %d ==============",lookahead, i);
//       // Solve the motion planning problem
//       ompl::base::PlannerStatus status;
//       status = planner.solve(ompl::base::plannerNonTerminatingCondition());
//
//       // ==================== record the results ========================//
//       totalNumberOfEdgeEvaluation = totalNumberOfEdgeEvaluation + planner.getNumberOfEdgeEvaluations();
//       totalNumberOfVertexExpansion = totalNumberOfVertexExpansion + planner.getNumberOfVertexExpansions();
//
//       if (planner.getNumberOfEdgeEvaluations()!=0)
//       totalEdgeEvaluationTime = totalEdgeEvaluationTime
//               + (planner.getNumberOfEdgeEvaluations()*planner.getAvgEdgeEvalTime() );
//
//       if (planner.getNumberOfVertexExpansions()!=0)
//       totalVertexExpansionTime = totalVertexExpansionTime
//               + (planner.getNumberOfVertexExpansions()*planner.getAvgVertexExpTime() );
//
//       totalTimeSpent = totalEdgeEvaluationTime+totalVertexExpansionTime;
//
//       myfile <<  planner.getNumberOfVertices()-2 << ",";
//       myfile <<  planner.getNumberOfEdgeEvaluations() << ",";
//       myfile <<  planner.getNumberOfVertexExpansions() << ",";
//       myfile <<  planner.getBestPathCost() << ",";
//       myfile <<  planner.getAvgEdgeEvalTime() << ",";
//       myfile <<  planner.getAvgVertexExpTime() << ",";
//       myfile <<  totalNumberOfEdgeEvaluation << ",";
//       myfile <<  totalNumberOfVertexExpansion << ",";
//       myfile <<  totalEdgeEvaluationTime << ",";
//       myfile <<  totalVertexExpansionTime << ",";
//       myfile <<  totalTimeSpent << "\n";
//
//       // Obtain required data if plan was successful
//       if (status == ompl::base::PlannerStatus::EXACT_SOLUTION) {
//         // Display path and specify path size
//         // auto path = std::dynamic_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
//         std::cout << "Solution Path Cost: " << planner.getBestPathCost() << std::endl;
//         std::cout << "Number of Edge Evaluations: " << planner.getNumberOfEdgeEvaluations()
//                   << std::endl;
//         std::cout << "Number of Vertex Expansions: " << planner.getNumberOfVertexExpansions()
//                   << std::endl;
//
//         // displayTree(obstacleLocation1, 1000, planner.getGraph());
//         // planner.setInflationFactor(inflationFactor);
//       }
//
//       planner.generateNewSamples(batchSize,true);
//
//     } // End for loop of iterations
//
//     // Now close the file
//     myfile.close();
//
//   } // End of for loop lookahead

  return 0;
}

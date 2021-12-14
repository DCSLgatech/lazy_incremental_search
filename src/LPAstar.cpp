/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#include "lgls/LPAstar.hpp"

#include <cmath>    // pow, sqrt
#include <iostream> // std::invalid_argument
#include <set>      // std::set
#include <assert.h> // debug

#include <boost/graph/connected_components.hpp> // connected_components

using lgls::datastructures::Keys;
using lgls::datastructures::CollisionStatus;
using lgls::datastructures::Edge;
using lgls::datastructures::EdgeIter;
using lgls::datastructures::EdgeProperties;
using lgls::datastructures::EPLengthMap;
using lgls::datastructures::EvaluationStatus;
using lgls::datastructures::Graph;
using lgls::datastructures::NeighborIter;
using lgls::datastructures::Path;
using lgls::datastructures::State;
using lgls::datastructures::StatePtr;
using lgls::datastructures::Vertex;
using lgls::datastructures::VertexIter;
using lgls::datastructures::VertexProperties;
using lgls::datastructures::VPStateMap;


namespace lgls {

LPAstar::LPAstar(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::Planner(si, "LPAstar"), mSpace(si->getStateSpace()) {
  // Set default values for data members.
}

LPAstar::~LPAstar() {
  // Do nothing.
}

// ============================================================================
void LPAstar::setup() {
  // Check if already setup.
  if (static_cast<bool>(ompl::base::Planner::setup_))
    return;

  // Mark the planner to have been setup.
  ompl::base::Planner::setup();

  // Check if the graph has been setup.
  if (!mGraphSetup)
    std::invalid_argument("Graph has not been provided.");

  // TODO (avk): If the graph is not provided, use implicit representation
  // for the edges using the NearestNeighbor representation.
  // Check if roadmap has been provided.

  OMPL_INFORM("Planner has been setup.");
}

// ============================================================================
void LPAstar::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) {
  // Make sure we setup the planner first.
  if (!static_cast<bool>(ompl::base::Planner::setup_)) {
    setup();
  }

  // Mark the planner's problem to be defined.
  ompl::base::Planner::setProblemDefinition(pdef);

  setupPreliminaries();
  OMPL_INFORM("Problem Definition has been setup.");
}

// ============================================================================
void LPAstar::setupPreliminaries() {
  // Issue a warning if mConnectionRadius = 0.

  // TODO (avk): Should I kill these pointers manually?
  StatePtr sourceState(new lgls::datastructures::State(mSpace));
  mSpace->copyState(sourceState->getOMPLState(), pdef_->getStartState(0));

  StatePtr targetState(new lgls::datastructures::State(mSpace));
  mSpace->copyState(
      targetState->getOMPLState(), pdef_->getGoal()->as<ompl::base::GoalState>()->getState());

  // Add start and goal vertices to the graph
  mSourceVertex = boost::add_vertex(mGraph);
  mGraph[mSourceVertex].setState(sourceState);

  mTargetVertex = boost::add_vertex(mGraph);
  mGraph[mTargetVertex].setState(targetState);

  // Assign default values.
  mGraph[mSourceVertex].setRHS(0);
  mGraph[mSourceVertex].setCostToCome(std::numeric_limits<double>::infinity());
  mGraph[mSourceVertex].setHeuristic(getGraphHeuristic(mSourceVertex));
  mGraph[mSourceVertex].setParent(mSourceVertex);

  mGraph[mTargetVertex].setRHS(std::numeric_limits<double>::infinity());
  mGraph[mTargetVertex].setCostToCome(std::numeric_limits<double>::infinity());
  mGraph[mTargetVertex].setHeuristic(0);

  // TODO (AVK): Make this kNN + R-disc. Additionally join the start and goal.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    double sourceDistance
        = mSpace->distance(mGraph[*vi].getState()->getOMPLState(), sourceState->getOMPLState());
    double targetDistance
        = mSpace->distance(mGraph[*vi].getState()->getOMPLState(), targetState->getOMPLState());

    if (sourceDistance < mConnectionRadius) {
      if (mSourceVertex == *vi)
        continue;

      std::pair<Edge, bool> newEdge = boost::add_edge(mSourceVertex, *vi, mGraph);

      mGraph[newEdge.first].setLength(sourceDistance);
      assert(newEdge.second);
    }

    if (targetDistance < mConnectionRadius) {
      if (mTargetVertex == *vi)
        continue;

      std::pair<Edge, bool> newEdge = boost::add_edge(mTargetVertex, *vi, mGraph);
      mGraph[newEdge.first].setLength(targetDistance);
      assert(newEdge.second);
    }
  }

  // Additionally connect the source and target with a straight line to snap.
  // std::pair<Edge, bool> newEdge = boost::add_edge(mSourceVertex, mTargetVertex, mGraph);
  // mGraph[newEdge.first].setLength(
  //     mSpace->distance(sourceState->getOMPLState(), targetState->getOMPLState()));
  // mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
  // mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);

  // Initialize the search
  this->updateVertex(mSourceVertex);

}

// ============================================================================
void LPAstar::clear() {
  // Call the base clear
  ompl::base::Planner::clear();

  // Clear the queues.
  mQueue.clear();
  assert(mQueue.isEmpty());


  // Reset the vertices and edges.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    mGraph[*vi].setCostToCome(std::numeric_limits<double>::infinity());
    mGraph[*vi].setRHS(std::numeric_limits<double>::infinity());
    mGraph[*vi].setHeuristic(std::numeric_limits<double>::infinity());
  }

  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(mGraph); ei != ei_end; ++ei) {
    mGraph[*ei].setEvaluationStatus(EvaluationStatus::NotEvaluated);
  }

  // Remove edges between source, target to other vertices.
  clear_vertex(mSourceVertex, mGraph);
  clear_vertex(mTargetVertex, mGraph);

  // Remove the vertices themselves.
  remove_vertex(mSourceVertex, mGraph);
  remove_vertex(mTargetVertex, mGraph);

  setBestPathCost(std::numeric_limits<double>::infinity());
  mNumberOfEdgeEvaluations = 0;
  mNumberOfVertexExpansions = 0;
  mPlannerStatus = PlannerStatus::NotSolved;


  OMPL_INFORM("Cleared Everything");
}

// ============================================================================
void LPAstar::setDebugCallback(std::function<void(Graph)> callback) {
    mCallback = callback;
}

// ============================================================================
ompl::base::PlannerStatus LPAstar::solve(const ompl::base::PlannerTerminationCondition& /*ptc*/) {
  // TODO (avk): Use ptc to terminate the search.

  // Visualization thread clock
  mJoined = false;
  std::thread visualize_thread{[this]() {call_visualize(); }};

  /// Let's see if source or target are in collision
  if (this->evaluateVertex(mSourceVertex) == CollisionStatus::Collision) {
    OMPL_INFORM("Start State is invalid.");
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (this->evaluateVertex(mTargetVertex) == CollisionStatus::Collision) {
    OMPL_INFORM("Goal State is invalid.");
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  // Main search
  this->computeShortestPath();

  // Visualization clock thread joined
  mJoined = true;
  visualize_thread.join();

  // Report the timing average results
  std::cout << "Average Time to evaluate an edge : " << mTotalEdgeEvaluationTime/mNumberOfEdgeEvaluations << " s" <<std::endl;
  std::cout << "Average Time to exapnd a vertex : " << mTotalVertexExpansionTime/mNumberOfVertexExpansions << " s" <<std::endl;


  if (mPlannerStatus == PlannerStatus::Solved) {
    this->setBestPathCost(mGraph[mTargetVertex].getCostToCome());
    pdef_->addSolutionPath(this->constructSolution(mSourceVertex, mTargetVertex));
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  } else {
    OMPL_INFORM("No Solution Found.");
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}

// ============================================================================
Keys LPAstar::calculateKeys(Vertex u) {
  double minval = std::min(mGraph[u].getCostToCome(),mGraph[u].getRHS());
  return std::make_pair(minval+this->getGraphHeuristic(u), minval);

  // TODO: (jil) Do we need to store vertex costToGo heurstic?
  // return std::make_pair(minval+mGraph[u].getHeuristic(), minval);
}

// ============================================================================
// TODO : upgrade this to new version -> see TLPAstar for the way it handles tempRHS
void LPAstar::updateVertex(Vertex v) {
  // Don't find an optimal parent for start vertex
  if (v != mSourceVertex) {

    // Temporary data holder to find an optimal parent
    Vertex newParent;
    double tempRHS =  std::numeric_limits<double>::infinity();

    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(v, mGraph); ni != ni_end; ++ni) {
      Vertex u = *ni;

      // Get the edge between the two vertices.
      Edge uv = this->getEdge(u, v);

      // Now is the good time to evaluate the edge if we haven't
      if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated) {

        this->evaluateEdge(uv);

      } // End If evaluationStatusCheck

      if (mGraph[u].getCostToCome() + mGraph[uv].getValue() < tempRHS &&
          mGraph[u].getParent()!= v) {
            tempRHS = mGraph[u].getCostToCome() + mGraph[uv].getValue();
            newParent = u;
      }


    } // End For neighbor

    // Actual assignment to the new optimal parent
    if (std::isinf(tempRHS))
    {
      mGraph[v].setRHS(tempRHS);
      mGraph[v].removeParent();
    }
    else
    {
      mGraph[v].setRHS(tempRHS);
      mGraph[v].setParent(newParent);
    }

  } // End if non-source vertex

  // Now enqueue this vertex
  // First, Remove this vertex in the queue
  mQueue.removeVertex(v);

  // Insert this if it is inconsistent
  if(mGraph[v].getCostToCome()!=mGraph[v].getRHS())
    mQueue.addVertexWithKeys(v, this->calculateKeys(v));

}

// ============================================================================
void LPAstar::computeShortestPath() {

  while ( mQueue.keyComparison(mQueue.getTopVertexKeys(), this->calculateKeys(mTargetVertex)) ||
  mGraph[mTargetVertex].getRHS()!=mGraph[mTargetVertex].getCostToCome() ) {


    // Claim the mutex to use the graph
    std::unique_lock<std::mutex> lck{mtx};

    // Wait until finish drawing the graph
    cv.wait(lck);


    // For visualization only, safe to comment out or remove if not needed.
    // if(mCallback && mDraw) mCallback(mGraph);
    // mDraw = false; // Set this off, so that the clock can turn on

    auto tic = std::chrono::high_resolution_clock::now();

    // check if the queue is empty.. which means graph has no connected path
    if (mQueue.isEmpty()){
      OMPL_INFORM("No Path Exists in the graph");
      std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        << ", Edge Evaluated: "<< mNumberOfEdgeEvaluations
        << ", Queue Size: " <<  mQueue.getSize() << std::endl;

      lck.unlock();
      return;
    }

    // Pop front vertex from the queue
    Vertex frontVertex = mQueue.popTopVertex();

    // Count the number of expansion
    mNumberOfVertexExpansions++;

    // Is it overconsistent?
    if (mGraph[frontVertex].getCostToCome() >mGraph[frontVertex].getRHS() ) {
      // Make it consistent
      mGraph[frontVertex].setCostToCome(mGraph[frontVertex].getRHS());
    }
    else {
      // Otherwise it is underconsistent, no consistent vertices are in the queue
      // Make it overconsistent or consistent
      mGraph[frontVertex].setCostToCome(std::numeric_limits<double>::infinity());

      // Update this vertex
      this->updateVertex(frontVertex);

    }
    // Now update the sucessor vertices
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(frontVertex, mGraph); ni != ni_end; ++ni) {
      Vertex u = *ni;
      this->updateVertex(u);
    } // End for successor vertices

    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    mTotalVertexExpansionTime += time_span.count();


    lck.unlock();
  } // End while loop

  mPlannerStatus = PlannerStatus::Solved;
  // mQueue.printQueue();
}

// ============================================================================
void LPAstar::perceiveNewWorld() {
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    // This is for identifying changed vertices.... only for approximate change detection
    this->evaluateVertex(*vi);
  } // End For vertex iteration
}


bool LPAstar::perceiveChanges() {

  // Hope no changes
  bool isChanged = false;

  // Flush out the previously perceived changes
  mPerceivedChangedEdges.clear();

  // Reset counters;
  mNumberOfEdgeEvaluations=0;

  mNumberOfVertexExpansions=0;

  mTotalEdgeEvaluationTime=0;

  mTotalVertexExpansionTime=0;


  std::vector<Vertex> verticesTobeUpdated;

  //////////////////////// Vertex Approximate Evaluation /////////////////
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {

    // Go through each vertex, to detect whether this has been changed.
    CollisionStatus oldStartColor = mGraph[*vi].getCollisionStatus();

    if (this->evaluateVertex(*vi) != oldStartColor )
    {
      // Yes, this vertex is different than before.
      // Therefore all the incident edges need to be checked.
      // Collect all incident edges.
      NeighborIter ni, ni_end;
      for (boost::tie(ni, ni_end) = adjacent_vertices(*vi, mGraph); ni != ni_end; ++ni)
      {

        Vertex u = *ni;
        // Get the edge between the two vertices.
        Edge edge = this->getEdge(u, *vi);

        mPerceivedChangedEdges.push_back(edge);

      }//End For neighboring edges

    } //End If vertex change

  } // End For vertex iteration

  // So far, we have collected candidate edges that could have been changed.
  std::cout << "Perceives possible "<< mPerceivedChangedEdges.size() <<" edges change" << std::endl;

  // Now go through the candidate edges, and check if it did change.
  for (std::vector<Edge>::iterator it = mPerceivedChangedEdges.begin() ; it != mPerceivedChangedEdges.end(); ++it)
  {
    // No need to insert if it is already unevaluated
    if (mGraph[*it].getEvaluationStatus() != EvaluationStatus::NotEvaluated)
    {

      // Now is the time to evaluate
      CollisionStatus previousEdgeColor = mGraph[*it].getCollisionStatus();
      // Did it really change?
      if (previousEdgeColor!=this->evaluateEdge(*it))
      {
        // yes, indeed this edge is different. Collect all the vertices to update once
        Vertex startVertex = source(*it, mGraph);

        Vertex endVertex = target(*it, mGraph);

        if (std::find(verticesTobeUpdated.begin(), verticesTobeUpdated.end(), startVertex) == verticesTobeUpdated.end())
        {verticesTobeUpdated.push_back(startVertex);}

        if (std::find(verticesTobeUpdated.begin(), verticesTobeUpdated.end(), endVertex) == verticesTobeUpdated.end())
        {verticesTobeUpdated.push_back(endVertex);}

      } // End If edge changed

    }// End If previously evaluated
  } // End For going through candidate edges

  std::cout <<  verticesTobeUpdated.size() <<" vertices to be updated" << std::endl;

  if (!verticesTobeUpdated.empty()){

    // Now update the vertices
    for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it) {

      // Important : Make sure if the source Vertex is changed, then make it inconsistent
      if (*it == mSourceVertex) mGraph[*it].setCostToCome(std::numeric_limits<double>::infinity());

      // Assert that all changed edges are evaluated.
      this->updateVertex(*it);
    }

    // Okay, there is some change, we should re-solve it.
    isChanged = true;

    mPlannerStatus = PlannerStatus::NotSolved;

    pdef_->clearSolutionPaths();
  }

  return isChanged;
}

// ============================================================================
Edge LPAstar::getEdge(Vertex u, Vertex v) {
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);
  assert(edgeExists);

  return uv;
}

// ============================================================================
// TODO (avk): I should be able to set the heuristic function from the demo
// script. Create a Heuristic Class and send it in. Have a default heuristic
// if nothing has been set.
double LPAstar::getGraphHeuristic(Vertex v) {
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());
  return heuristic;
}

// ============================================================================
void LPAstar::setConnectionRadius(double radius) {
  mConnectionRadius = radius;
}

// ============================================================================
double LPAstar::getConnectionRadius() {
  return mConnectionRadius;
}

// ============================================================================
void LPAstar::setCollisionCheckResolution(double resolution) {
  mCollisionCheckResolution = resolution;
}

// ============================================================================
double LPAstar::getCollisionCheckResolution() {
  return mCollisionCheckResolution;
}

// ============================================================================
void LPAstar::setRoadmap(std::string filename) {
  if (filename == "")
    std::invalid_argument("Roadmap Filename cannot be empty!");

  // Load the graph.
  mRoadmap = boost::shared_ptr<io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>>(
      new io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>(mSpace, filename));

  mRoadmap->generate(
      mGraph, get(&VertexProperties::mState, mGraph), get(&EdgeProperties::mLength, mGraph));

  // Mark the graph to have been setup.
  mGraphSetup = true;
}

// ============================================================================
void LPAstar::setBestPathCost(double cost) {
  mBestPathCost = cost;
}

// ============================================================================
double LPAstar::getBestPathCost() {
  return mBestPathCost;
}

// ============================================================================
void LPAstar::setPlannerStatus(PlannerStatus status) {
  mPlannerStatus = status;
}

// ============================================================================
PlannerStatus LPAstar::getPlannerStatus() {
  return mPlannerStatus;
}

// ============================================================================
double LPAstar::getNumberOfEdgeEvaluations() {
  return mNumberOfEdgeEvaluations;
}

// ============================================================================
double LPAstar::getNumberOfVertexExpansions() {
  return mNumberOfVertexExpansions;
}

// ============================================================================
CollisionStatus LPAstar::evaluateVertex(const Vertex& v) {
  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();

  auto state = mGraph[v].getState()->getOMPLState();

  // Evaluate the state.
  if (!validityChecker->isValid(state))
      return CollisionStatus::Collision;

  return CollisionStatus::Free;
}

// ============================================================================
CollisionStatus LPAstar::evaluateEdge(const Edge& e) {

  auto tic = std::chrono::high_resolution_clock::now();

  mNumberOfEdgeEvaluations++;

  // Be optimisitic before evaluation... This will be free, this will be free...
  CollisionStatus edgeColor =  CollisionStatus::Free;
  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();

  // Collision check the start and goal.
  Vertex startVertex = source(e, mGraph);
  Vertex endVertex = target(e, mGraph);

  auto startState = mGraph[startVertex].getState()->getOMPLState();
  auto endState = mGraph[endVertex].getState()->getOMPLState();

  // Evaluate Start and End States.
  if (!validityChecker->isValid(startState)) {
    mGraph[startVertex].setCollisionStatus(CollisionStatus::Collision);
    edgeColor = CollisionStatus::Collision;
  }
  else if (!validityChecker->isValid(endState)) {
    mGraph[endVertex].setCollisionStatus(CollisionStatus::Collision);
    edgeColor = CollisionStatus::Collision;
  }

  // Did it pass the end Vertices test?
  if (edgeColor  != CollisionStatus::Collision ) {
    // Evaluate the state in between.
    int maxSteps = 1.0 / mCollisionCheckResolution;
    for (int multiplier = 1; multiplier < maxSteps + 1; ++multiplier) {
      double interpolationStep = mCollisionCheckResolution * multiplier;
      assert(interpolationStep <= 1);
      StatePtr midVertex(new lgls::datastructures::State(mSpace));
      mSpace->interpolate(startState, endState, interpolationStep, midVertex->getOMPLState());

      if (!validityChecker->isValid(midVertex->getOMPLState())){
          edgeColor = CollisionStatus::Collision;
          break;
      }
    } // End For interpolation
  } // End If edge still free

  // Now assign the edge value
  double edgeValue;
  if (edgeColor == CollisionStatus::Collision) {
    edgeValue = std::numeric_limits<double>::infinity();
  }
  else {
    edgeValue = mSpace->distance(startState, endState);
  }

  // Actual assignment
  mGraph[e].setValue(edgeValue);
  mGraph[e].setEvaluationStatus(EvaluationStatus::Evaluated);

  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
  mTotalEdgeEvaluationTime += time_span.count();

  return edgeColor;

}


// ============================================================================
ompl::base::PathPtr LPAstar::constructSolution(const Vertex& source, const Vertex& target) {
  ompl::geometric::PathGeometric* path = new ompl::geometric::PathGeometric(si_);
  Vertex v = target;
  // std::cout << "path : " ;
  while (v != source) {
    // std::cout << v << ", " ;
    path->append(mGraph[v].getState()->getOMPLState());
    v = mGraph[v].getParent();
  }
  // std::cout << std::endl;
  if (v == source) {
    path->append(mGraph[source].getState()->getOMPLState());
  }
  path->reverse();
  return ompl::base::PathPtr(path);
}


// // ============================================================================
// void LPAstar::call_visualize()
// {
//   std::unique_lock<std::mutex> lck{mtx};
//   // Run this clock until joined
//   while(!mJoined)
//   {
//     cv.wait_for(lck, time);
//     // Now, draw the graph!
//     mDraw = true;
//   }
// }

// ============================================================================
void LPAstar::call_visualize()
{

  // Run this clock until joined
  while(!mJoined)
  {
    // wait for a few miliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    std::unique_lock<std::mutex> lck{mtx};

    // Now, draw the graph!
    if(mCallback) mCallback(mGraph);

    // Done drawing, let go of the lock
    lck.unlock();
    // I am done drawing, notify the main solve thread to continue;
    cv.notify_one();
  }
}


} // namespace lgls

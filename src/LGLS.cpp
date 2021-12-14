/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#include "lgls/LGLS.hpp"

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
using lgls::event::ConstEventPtr;
using lgls::event::EventPtr;

namespace lgls {

LGLS::LGLS(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::Planner(si, "LGLS"), mSpace(si->getStateSpace()) {
  // Set default values for data members.
    mHaltonSequence = std::make_shared<ompl::base::HaltonSequence>(si->getStateDimension());
}

LGLS::~LGLS() {
  // Do nothing.
}

// ============================================================================
void LGLS::setup() {
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
void LGLS::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) {
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
void LGLS::setupPreliminaries() {

  setupKNN();
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

  // Add to nearest neighbor structure
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    knnGraph.add(*vi);
  } // End For vertex iteration

  std::vector<Vertex> nearestSource;
  std::vector<Vertex> nearestTarget;

  // Add nearest vertices around the source to the graph
  knnGraph.nearestR(mSourceVertex, mConnectionRadius, nearestSource);

  Edge uv;
  bool edgeExists;
  for (const auto& v : nearestSource) {
    // skip the source vertex itself
    if (mSourceVertex == v)
      continue;
    double distance = mSpace->distance(
        mGraph[v].getState()->getOMPLState(), mGraph[mSourceVertex].getState()->getOMPLState());
    boost::tie(uv, edgeExists) = edge(mSourceVertex, v, mGraph);
    if (!edgeExists){
        std::pair<Edge, bool> newEdge = boost::add_edge(mSourceVertex, v, mGraph);
        mGraph[newEdge.first].setLength(distance);
        mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        assert(newEdge.second);
    }
  }


  // Add nearest vertices around the target to the graph
  knnGraph.nearestR(mTargetVertex, mConnectionRadius, nearestTarget);

  for (const auto& v : nearestTarget) {
    // skip the target vertex itself
    if (mTargetVertex == v)
      continue;
    double distance = mSpace->distance(
        mGraph[v].getState()->getOMPLState(), mGraph[mTargetVertex].getState()->getOMPLState());
    boost::tie(uv, edgeExists) = edge(mTargetVertex, v, mGraph);
    if (!edgeExists){
        std::pair<Edge, bool> newEdge = boost::add_edge(mTargetVertex, v, mGraph);
        mGraph[newEdge.first].setLength(distance);
        mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        assert(newEdge.second);
    }
  }


  // Additionally connect the source and target with a straight line to snap.
  // std::pair<Edge, bool> newEdge = boost::add_edge(mSourceVertex, mTargetVertex, mGraph);
  // mGraph[newEdge.first].setLength(
  //     mSpace->distance(sourceState->getOMPLState(), targetState->getOMPLState()));
  // mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
  // mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);

  // Setup the event.
  mEvent->setup(&mGraph, mSourceVertex, mTargetVertex);

}

// ============================================================================
void LGLS::clear() {
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
    mGraph[*vi].setEvaluationStatus(EvaluationStatus::NotEvaluated);
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

  // TODO(avk): Clear the selector and event.

  OMPL_INFORM("Cleared Everything");
}

// ============================================================================
void LGLS::setDebugCallback(std::function<void(Graph)> callback) {
    mCallback = callback;
}

// ============================================================================
ompl::base::PlannerStatus LGLS::solve(const ompl::base::PlannerTerminationCondition& /*ptc*/) {
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

  // Initialize the search
  this->updateVertex(mSourceVertex);

  // Run in loop.
  while (mPlannerStatus != PlannerStatus::Solved) {

    // Claim the mutex to use graph
    std::unique_lock<std::mutex> lck{mtx};

    // Wait until finish drawing the graph
    cv.wait(lck);

    /// Repair the tree till the event is triggered. returns the leaf
    Vertex triggeredLeaf;
    if(this->computeShortestPath(triggeredLeaf))
    {

      // std::cout << "Lazy LPA* returns a path "<< triggeredLeaf << std::endl;
      /// Evaluate along the subpath to the leaf, returns the first inconsis edge
      Edge inconsistentEdge ;
      bool inconsistencyExist = this->evaluatePath(triggeredLeaf,inconsistentEdge);

      // std::cout << "  evauated this path"<< triggeredLeaf << std::endl;

      /// Let the lazy LPA* handle the inconsistency
      if (inconsistencyExist) {
        this->updateVertex(target(inconsistentEdge, mGraph));
        // std::cout<< "inconsistent edge found updated" <<std::endl;
      }
      else
      {
        // NO inconsistent edge is found,
        if (triggeredLeaf == mTargetVertex)
        {
          mPlannerStatus = PlannerStatus::Solved;
        }
      }

    } else
    {
      // No triggering vertex exists
      OMPL_INFORM("No Trigerring Vertex Exists in the graph");
      std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        << ", Edge Evaluated: "<< mNumberOfEdgeEvaluations
        << ", Queue Size: " <<  mQueue.getSize() << std::endl;
      break;
    }


  } // End while loop

  // Visualization clock thread joined
  mJoined = true;
  visualize_thread.join();

  // Report the timing average results
  std::cout << "Average Time to evaluate an edge : " << mTotalEdgeEvaluationTime/mNumberOfEdgeEvaluations << " s" <<std::endl;
  std::cout << "Average Time to exapnd a vertex : " << mTotalVertexExpansionTime/mNumberOfVertexExpansions << " s" <<std::endl;


  if (mPlannerStatus == PlannerStatus::Solved && !std::isinf(mGraph[mTargetVertex].getCostToCome())) {
    setBestPathCost(mGraph[mTargetVertex].getCostToCome());
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  } else {
    OMPL_INFORM("No Solution Found.");
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}

// ============================================================================
Keys LGLS::calculateKeys(Vertex u) {
  double minval = std::min(mGraph[u].getCostToCome(),mGraph[u].getRHS());
  return std::make_pair(minval+this->getGraphHeuristic(u), minval);

}

// ============================================================================
void LGLS::updateVertex(Vertex v) {
  // Don't find an optimal parent for start vertex
  if (v != mSourceVertex) {

    // Temporary data holder to find an optimal parent
    Vertex newParent = boost::graph_traits<lgls::datastructures::BasicGraph>::null_vertex();
    double tempRHS =  std::numeric_limits<double>::infinity();

    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(v, mGraph); ni != ni_end; ++ni) {
      Vertex u = *ni;

      // Get the edge between the two vertices.
      Edge uv = this->getEdge(u, v);

      if (mGraph[u].getCostToCome() + mGraph[uv].getValue() < tempRHS
        //&& mGraph[u].getParent()!= v) {
        )
      {
            tempRHS = mGraph[u].getCostToCome() + mGraph[uv].getValue();
            newParent = u;
      }


    } // End For neighbor

    // Actual assignment to the new optimal parent
    mGraph[v].setRHS(tempRHS);
    mGraph[v].setParent(newParent);

  } // End if non-source vertex

  // Now enqueue this vertex
  // First, Remove this vertex in the queue
  mQueue.removeVertex(v);

  // Insert this if it is inconsistent
  if(mGraph[v].getCostToCome()!=mGraph[v].getRHS())
    mQueue.addVertexWithKeys(v, this->calculateKeys(v));

}

// ============================================================================
bool LGLS::computeShortestPath(Vertex& triggeredLeafVertex) {

  while (mQueue.keyComparison(mQueue.getTopVertexKeys(), this->calculateKeys(mTargetVertex)) ||
  mGraph[mTargetVertex].getRHS()!=mGraph[mTargetVertex].getCostToCome() ) {

    // For visualization only, safe to comment out or remove if not needed.
    // if(mCallback) mCallback(mGraph);

    auto tic = std::chrono::high_resolution_clock::now();

    if (mQueue.isEmpty()){
      OMPL_INFORM("No Path Exists in the graph");
      std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        << ", Edge Evaluated: "<< mNumberOfEdgeEvaluations
        << ", Queue Size: " <<  mQueue.getSize() << std::endl;

      return false;
    }
    // Pop front vertex from the queue
    Vertex frontVertex = mQueue.popTopVertex();


    // Count the number of expansion
    mNumberOfVertexExpansions++;


    // Is it overconsistent?
    if (mGraph[frontVertex].getCostToCome() >mGraph[frontVertex].getRHS() ) {

      // Yes, Make it consistent
      mGraph[frontVertex].setCostToCome(mGraph[frontVertex].getRHS());

      // Right after making it consistent, check if this triggers event
      if (mEvent->isTriggered(frontVertex)) {
        triggeredLeafVertex = frontVertex;
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
        mTotalVertexExpansionTime += time_span.count();

        return true;
      }// No else, continue repairing.

    }
    else {
      // Otherwise it is underconsistent, since no consistent vertices are in the queue

      // Make it overconsistent (or consistent when rhs=inf)
      mGraph[frontVertex].setCostToCome(std::numeric_limits<double>::infinity());

      // Update this vertex
      this->updateVertex(frontVertex);
    }

    // Now update the sucessor vertices
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(frontVertex, mGraph); ni != ni_end; ++ni) {
      Vertex u = *ni;
      this->updateVertex(u);
      // std::cout << "updates neighbor: " << u << ", Queue :" << mQueue.getSize() <<std::endl;
    } // End for successor vertices

    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    mTotalVertexExpansionTime += time_span.count();

  } // End while loop

  // Okay, the tree seems to be consistent all the way up to the goal.
  // Let's return the goal if it is connected
  triggeredLeafVertex = mTargetVertex;

  if (mGraph[triggeredLeafVertex].hasParent())
  return true;

  // Otherwise, the goal is not in connected component
  return false;

}

// ============================================================================
void LGLS::perceiveNewWorld() {
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    // This is for identifying changed vertices.... only for approximate change detection
    this->evaluateVertex(*vi);
  } // End For vertex iteration

  OMPL_INFORM("First time world is seen");
}


bool LGLS::perceiveChanges() {

  // Hope no changes
  bool isChanged = false;

  // Subset of edges that have been changed.
  std::vector<lgls::datastructures::Edge> perceivedChangedEdges;

  // Reset counters;
  mNumberOfEdgeEvaluations=0;

  mNumberOfVertexExpansions=0;

  mTotalEdgeEvaluationTime=0;

  mTotalVertexExpansionTime=0;

  std::vector<Vertex> verticesTobeUpdated;

  //////////////////////// Vertex Approximate Evaluation //////////////////////
  /// The idea is to approximate the edge set that has been evaluated       ///
  /// and CollisionStatus has changed.                                      ///
  /// 1. check whether a vertex has been evaluated,                         ///
  ///    if not, then the adjacent edges are necesseraily unevaluated.      ///
  /// 2. Among the evaluated vertices, check if the CollisionStatus         ///
  ///    has changed.                                                       ///
  /// 3. Check the adjacent edges that have been evaluated.                 ///
  /// 4. Given that either one end vertex changed the CollisionStatus,      ///
  ///    that edge could have been changed.                                 ///
  /////////////////////////////////////////////////////////////////////////////
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {

    // Go through each vertex, to detect whether this has been changed.
    // No need to check vertices that were not evaluated
    if (mGraph[*vi].getEvaluationStatus() == EvaluationStatus::Evaluated)
    {
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
          if(mGraph[edge].getEvaluationStatus() == EvaluationStatus::Evaluated)
            perceivedChangedEdges.push_back(edge);

        }//End For neighboring edges

      } //End If vertex change

    } // End If vertex evaluated
  } // End For vertex iteration

  // So far, we have collected candidate edges that could have been changed.
  std::cout << "Perceives possible "<< perceivedChangedEdges.size() <<" edges change" << std::endl;

  // Now go through the candidate edges, and check if it did change.
  for (std::vector<Edge>::iterator it = perceivedChangedEdges.begin() ; it != perceivedChangedEdges.end(); ++it)
  {

        // Note that setting this NotEvaluated will make the returned edge value its length.
        mGraph[*it].setEvaluationStatus(EvaluationStatus::NotEvaluated);

        // Collect all the vertices to update once
        Vertex startVertex = source(*it, mGraph);

        Vertex endVertex = target(*it, mGraph);

        if (std::find(verticesTobeUpdated.begin(), verticesTobeUpdated.end(), startVertex) == verticesTobeUpdated.end())
        {
          verticesTobeUpdated.push_back(startVertex);
          mGraph[startVertex].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        }

        if (std::find(verticesTobeUpdated.begin(), verticesTobeUpdated.end(), endVertex) == verticesTobeUpdated.end())
        {
          verticesTobeUpdated.push_back(endVertex);
          mGraph[endVertex].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        }

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
void LGLS::setEvent(EventPtr event) {
  mEvent = event;
}

// ============================================================================
ConstEventPtr LGLS::getEvent() const {
  return mEvent;
}

// ============================================================================
Edge LGLS::getEdge(Vertex u, Vertex v) {
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);
  assert(edgeExists);

  return uv;
}

// ============================================================================
Path LGLS::getPathToSource(Vertex u) {
  Path pathToSource;
  while (u != mSourceVertex) {
    pathToSource.emplace_back(u);
    assert(mGraph[u].hasParent());
    u = mGraph[u].getParent();
  }
  pathToSource.emplace_back(mSourceVertex);
  return pathToSource;
}

// ============================================================================
// TODO (avk): I should be able to set the heuristic function from the demo
// script. Create a Heuristic Class and send it in. Have a default heuristic
// if nothing has been set.
double LGLS::getGraphHeuristic(Vertex v) {
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());
  return heuristic;
}

// ============================================================================
void LGLS::setConnectionRadius(double radius) {
  mConnectionRadius = radius;
}

// ============================================================================
double LGLS::getConnectionRadius() {
  return mConnectionRadius;
}

// ============================================================================
void LGLS::setCollisionCheckResolution(double resolution) {
  mCollisionCheckResolution = resolution;
}

// ============================================================================
double LGLS::getCollisionCheckResolution() {
  return mCollisionCheckResolution;
}

// ============================================================================
void LGLS::setRoadmap(std::string filename) {
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
double LGLS::distFun(const lgls::datastructures::Vertex& v1, const lgls::datastructures::Vertex& v2) {
  mGraph[v1].getState()->getOMPLState();
  return mSpace->distance(
      mGraph[v1].getState()->getOMPLState(), mGraph[v2].getState()->getOMPLState());
}

// ============================================================================
void LGLS::setupKNN() {
  knnGraph.setDistanceFunction(
      std::bind(&LGLS::distFun, this, std::placeholders::_1, std::placeholders::_2));
}

// ============================================================================
void LGLS::setBestPathCost(double cost) {
  mBestPathCost = cost;
}

// ============================================================================
double LGLS::getBestPathCost() {
  return mBestPathCost;
}

// ============================================================================
void LGLS::setInflationFactor(double factor) {
  mInflationFactor = factor;
}

// ============================================================================
double LGLS::getInflationFactor() {
  return mInflationFactor;
}

// ============================================================================
void LGLS::setPlannerStatus(PlannerStatus status) {
  mPlannerStatus = status;
}

// ============================================================================
PlannerStatus LGLS::getPlannerStatus() {
  return mPlannerStatus;
}

// ============================================================================
double LGLS::getNumberOfEdgeEvaluations() {
  return mNumberOfEdgeEvaluations;
}

// ============================================================================
double LGLS::getNumberOfVertexExpansions() {
  return mNumberOfVertexExpansions;
}

// ============================================================================
CollisionStatus LGLS::evaluateVertex(const Vertex& v) {
  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();

  auto state = mGraph[v].getState()->getOMPLState();

  mGraph[v].setEvaluationStatus(EvaluationStatus::Evaluated);

  // Evaluate the state.
  if (!validityChecker->isValid(state))
  {
    mGraph[v].setCollisionStatus(CollisionStatus::Collision);
    return CollisionStatus::Collision;
  }

  mGraph[v].setCollisionStatus(CollisionStatus::Free);
  return CollisionStatus::Free;
}


// ============================================================================
CollisionStatus LGLS::evaluateEdge(const Edge& e) {

  auto tic = std::chrono::high_resolution_clock::now();

  mNumberOfEdgeEvaluations++;

  // Be optimisitc about the edge.
  CollisionStatus edgeColor = CollisionStatus::Free;

  // Collision check the start and goal.
  Vertex startVertex = source(e, mGraph);
  Vertex endVertex = target(e, mGraph);
  auto startState = mGraph[startVertex].getState()->getOMPLState();
  auto endState = mGraph[endVertex].getState()->getOMPLState();

  // Evaluate start and end vertices first.
  if (evaluateVertex(startVertex)==CollisionStatus::Collision)
  {
    edgeColor =  CollisionStatus::Collision;
  }

  if (evaluateVertex(endVertex)==CollisionStatus::Collision)
  {
    edgeColor =  CollisionStatus::Collision;
  }

  // Proceed to the intermideate states, if passed the start and end vertices check
  if (edgeColor != CollisionStatus::Collision)
  {
    // Access the validity checker.
    auto validityChecker = si_->getStateValidityChecker();

    // Evaluate the state in between.
    int maxSteps = 1.0 / mCollisionCheckResolution;
    for (int multiplier = 1; multiplier < maxSteps + 1; ++multiplier) {
      double interpolationStep = mCollisionCheckResolution * multiplier;
      assert(interpolationStep <= 1);
      StatePtr midVertex(new lgls::datastructures::State(mSpace));
      mSpace->interpolate(startState, endState, interpolationStep, midVertex->getOMPLState());

      if (!validityChecker->isValid(midVertex->getOMPLState())){
        edgeColor = CollisionStatus::Collision;
        break; // No need to check further
      }
    } // End For interpolation
  } // End If passed start-end check


  // Now assign the edge value
  double edgeValue;
  if (edgeColor == CollisionStatus::Collision) {
    edgeValue = std::numeric_limits<double>::infinity();
  }
  else {
    edgeValue = mSpace->distance(startState, endState);
  }

  // Actual assignment
  mGraph[e].setCollisionStatus(edgeColor);
  mGraph[e].setValue(edgeValue);
  mGraph[e].setEvaluationStatus(EvaluationStatus::Evaluated);

  auto toc = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
  mTotalEdgeEvaluationTime += time_span.count();

  return edgeColor;

}

// ============================================================================
bool LGLS::evaluatePath(const Vertex& triggeredLeaf, Edge& inconsistentEdge) {

  Path path2Evaluate = this->getPathToSource(triggeredLeaf);

  Edge edgeToEvaluate;

  // Return the first unevaluated edge closest to source.
  for (std::size_t i = path2Evaluate.size() - 1; i > 0; --i) {

    edgeToEvaluate= this->getEdge(path2Evaluate[i], path2Evaluate[i-1]);

    // Check if this edge has been not evaluated yet
    if (mGraph[edgeToEvaluate].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
      // getValue(mInflationFactor) will return
      // w_bar = { mInflationFactor*w_hat, if not evaluated,
      //         { w,                      otherwise.
      double oldVal = mGraph[edgeToEvaluate].getValue();
      // Now is the good time to evaluate
      this->evaluateEdge(edgeToEvaluate);

      // Is it inconsistent?
      if ( mGraph[edgeToEvaluate].getValue() != oldVal )
      {
        inconsistentEdge =  edgeToEvaluate;
        return true;
      }
      // No else, continue to the next unevaluated edge
    } // End If not evaluated edge

  } // End For path iteration

  // Nay, nothing special
  return false;

}

// ============================================================================
ompl::base::PathPtr LGLS::constructSolution(const Vertex& source, const Vertex& target) {
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


// ============================================================================
void LGLS::generateNewSamples(int batchSize, bool updateVertices) {

  // Reset counters;
  mNumberOfEdgeEvaluations=0;

  mNumberOfVertexExpansions=0;

  mTotalEdgeEvaluationTime=0;

  mTotalVertexExpansionTime=0;

  // Vertices to be updated if updateVertices is set True
  std::vector<Vertex> verticesTobeUpdated;
  // Collect near samples
  std::vector<Vertex> nearestSamples;

  // Validator
  auto validityChecker = si_->getStateValidityChecker();
  // Dimension of search space
  unsigned int dim = si_->getStateDimension();

  // Bounds for sampling
  const ompl::base::RealVectorBounds &bounds
    = static_cast<const ompl::base::RealVectorStateSpace*>(mSpace.get())->getBounds();

  // Scale to required limits.
  int numSampled = 0;

  // Sample start
  while (numSampled < batchSize) {

    // Generate a halton sample std::vector<double >
    auto newPosition = mHaltonSequence->sample();

    // Translate and scale to fit in the given bounds
    for (unsigned int i = 0; i < dim; ++i)
      newPosition[i] = bounds.low[i] + newPosition[i] * (bounds.high[i] - bounds.low[i]);

    // Our ompl::base::State* wrapper
    StatePtr sampledState(new lgls::datastructures::State(mSpace));

    // copy the Halton sequenced point to ompl::base::State*
    mSpace->copyFromReals(sampledState->getOMPLState(), newPosition);

    // skip it if not valid
    if(!validityChecker->isValid(sampledState->getOMPLState())) continue;

    // Since we have a valid sample, increment the numSampled.
    numSampled++;

    // Create a new vertex in the graph.
    Vertex sampleVertex = boost::add_vertex(mGraph);
    mGraph[sampleVertex].setState(sampledState);
    mGraph[sampleVertex].setEvaluationStatus(EvaluationStatus::Evaluated);
    mGraph[sampleVertex].setCollisionStatus(CollisionStatus::Free);
    // Do we need to assign default values?

    // Now add to the graph
    knnGraph.add(sampleVertex);
    verticesTobeUpdated.push_back(sampleVertex);

  } // End while a batch is sampled.

  // Update radius
  double connectionRadius = this->calculateR();
  std::cout << "current Connection Raidus: " << connectionRadius << std::endl;

  // Now Connect edges
  for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it)
  {
    // Collect near samples
    nearestSamples.clear();
    knnGraph.nearestR(*it, connectionRadius, nearestSamples);
    for (const auto& v : nearestSamples) {
      if(*it==v) continue;
      double distance = mSpace->distance(
          mGraph[v].getState()->getOMPLState(), mGraph[*it].getState()->getOMPLState());
      std::pair<Edge, bool> newEdge = boost::add_edge(*it, v, mGraph);
      mGraph[newEdge.first].setLength(distance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      assert(newEdge.second);
    }

  }

  // Update newly added vertices
  if (updateVertices)
  {
    // Now update the vertices
    for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it) {
      this->updateVertex(*it);
    }
    // Okay, there is some change, we should re-solve it.
    mPlannerStatus = PlannerStatus::NotSolved;

    pdef_->clearSolutionPaths();
  }


  OMPL_INFORM("A new batch of %d samples generated",batchSize);
}

double LGLS::calculateR() const
{
  // Cast to double for readability. (?)
  auto stateDimension = static_cast<double>(si_->getStateDimension());
  auto graphCardinality = static_cast<double>(boost::num_vertices(mGraph));
  // auto graphCardinality = static_cast<double>(mNumSamples);
  double approximateMesaure = si_->getSpaceMeasure();

  double minimumRGG = std::pow(2.0 * (1.0 + 1.0 / stateDimension) *
                      (approximateMesaure / ompl::unitNBallMeasure(si_->getStateDimension())),
                  1.0 / stateDimension);

  // Calculate the term and return.
  return mRewireFactor * minimumRGG *
         std::pow(std::log(graphCardinality) / graphCardinality, 1.0 / stateDimension);
}


// ============================================================================
void LGLS::call_visualize()
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

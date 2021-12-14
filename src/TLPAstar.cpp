/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#include "lgls/TLPAstar.hpp"

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

TLPAstar::TLPAstar(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::Planner(si, "TLPAstar"), mSpace(si->getStateSpace()) {
  // Set default values for data members.

  // Halton sequence sampler for n dimensional search
  mHaltonSequence = std::make_shared<ompl::base::HaltonSequence>(si->getStateDimension());

  // Uniform sampler
  mUniformSampler = si->allocStateSampler();
}

TLPAstar::~TLPAstar() {
  // Do nothing.
}

// ============================================================================
void TLPAstar::setup() {
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
void TLPAstar::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) {
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
void TLPAstar::setupPreliminaries() {
  // Issue a warning if mConnectionRadius = 0.

  setupKNN();

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
  mGraph[mSourceVertex].removeParent(); // Allows hasParent to return false; needed for computeGpi()
  mGraph[mSourceVertex].setGpi(0);

  mGraph[mTargetVertex].setRHS(std::numeric_limits<double>::infinity());
  mGraph[mTargetVertex].setCostToCome(std::numeric_limits<double>::infinity());
  mGraph[mTargetVertex].setHeuristic(0);
  mGraph[mTargetVertex].setGpi(std::numeric_limits<double>::infinity());

  // Add to nearest neighbor structure
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    knnGraph.add(*vi);
  } // End For vertex iteration

  // knnGraph.add(mSourceVertex);
  // knnGraph.add(mTargetVertex);

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

  // // Initialize the search
  // mQueue.addVertexWithKeys(mSourceVertex, this->calculateKeys(mSourceVertex));
  // if (mCallback) mCallback(mGraph);
}

// ============================================================================
void TLPAstar::clear() {
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
    mGraph[*vi].setGpi(std::numeric_limits<double>::infinity());
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
void TLPAstar::setDebugCallback(std::function<void(Graph)> callback) {
    mCallback = callback;
}

// ============================================================================
ompl::base::PlannerStatus TLPAstar::solve(const ompl::base::PlannerTerminationCondition& /*ptc*/) {
  // TODO (avk): Use ptc to terminate the search.

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

  // Main search
  /// Solve current graph using TLPA* search
  this->computeShortestPath();
  //OMPL_INFORM("TLPA* search completed.");

  // TODO : move these to PlannerData -- Report the timing average results
  std::cout << "Average Time to evaluate an edge : " << mTotalEdgeEvaluationTime/mNumberOfEdgeEvaluations << " s" <<std::endl;
  std::cout << "Average Time to exapnd a vertex : " << mTotalVertexExpansionTime/mNumberOfVertexExpansions << " s" <<std::endl;


  if (mPlannerStatus == PlannerStatus::Solved) {
    this->computeGpi(mTargetVertex);
    this->setBestPathCost(mGraph[mTargetVertex].getGpi());
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  } else {
    OMPL_INFORM("No Solution Found.");
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}

// ============================================================================
Keys TLPAstar::calculateKeys(Vertex u) {
  double minval = std::min(mGraph[u].getCostToCome(),mGraph[u].getRHS());
  return std::make_pair(minval+this->getGraphHeuristic(u), minval);

}


// ============================================================================
// Alg3: UpdateState
void TLPAstar::updateVertex(Vertex v) {
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

      // Now is the good time to evaluate the edge if we haven't
      if (mGraph[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated) {

        this->evaluateEdge(uv);

      } // End If evaluationStatusCheck

      if (mGraph[u].getCostToCome() + mGraph[uv].getValue() < tempRHS)
      {
            tempRHS = mGraph[u].getCostToCome() + mGraph[uv].getValue();
            newParent = u;
      }


    } // End For neighbor

    mGraph[v].setRHS(tempRHS);
    mGraph[v].setParent(newParent);

  } // End if non-source vertex

  // Now enqueue this vertex
  // First, Remove this vertex in the queue
  mQueue.removeVertex(v);

  // Insert this if it is inconsistent and not truncated
  if (!isTruncated(v))
  {
      // Reinsert if it is inconsistent
      if(mGraph[v].getCostToCome()!=mGraph[v].getRHS())
          mQueue.addVertexWithKeys(v, this->calculateKeys(v));
  }
}

// ============================================================================
// Alg 3: ComputePath
void TLPAstar::computeShortestPath() {

  while ( mQueue.keyComparison(mQueue.getTopVertexKeys(), this->calculateKeys(mTargetVertex)) ||
          mGraph[mTargetVertex].getRHS() > mGraph[mTargetVertex].getCostToCome() ) {

    auto tic = std::chrono::high_resolution_clock::now();

    // check if the queue is empty.. which means graph has no connected path
    if (mQueue.isEmpty()){
      OMPL_INFORM("No Path Exists in the graph");
      std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        << ", Edge Evaluated: " << mNumberOfEdgeEvaluations
        // << ", Queue Size: " <<  mQueue.getSize()
        << std::endl;
      return;
    }

    // Pop front vertex from the queue
    // Vertex s = mQueue.popTopVertex();

    // Don't pop yet,
    Vertex s = mQueue.getTopVertex();
    //std::cout << "Pop ";
    //printVertex(s);

    // Count the number of expansion
    mNumberOfVertexExpansions++;

    // Check truncated termination condition
    //std::cout << "T2: ";
    computeGpi(mTargetVertex);

    if (mGraph[mTargetVertex].getGpi() <= mTruncationFactor
          *(std::min(mGraph[s].getCostToCome(),mGraph[s].getRHS())+this->getGraphHeuristic(s) ) )
    {
      //std::cout << "T2! TLPA* terminated before expanding vertex " << s << std::endl;
      // Vertex Expansion timer off before exit
      auto toc = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
      mTotalVertexExpansionTime += time_span.count();

      mPlannerStatus = PlannerStatus::Solved;
      break;
    }

    // Now remove the top vertex from the queue
    mQueue.removeVertex(s);

    if (mGraph[s].getCostToCome() > mGraph[s].getRHS()) {
      // Make it consistent
      mGraph[s].setCostToCome(mGraph[s].getRHS());
      //std::cout << "Overconsistent! Make " << s << " consistent, update neighbors." << std::endl;

      // Count the number of expansion
      // mNumberOfVertexExpansions++;

      // Now update the sucessor vertices
      NeighborIter ni, ni_end;
      for (boost::tie(ni, ni_end) = adjacent_vertices(s, mGraph); ni != ni_end; ++ni)
        this->updateVertex(*ni);

    } // if overconsistent

    else {
      //std::cout << "T1: ";
      computeGpi(s);

      if(mGraph[s].getGpi() + this->getGraphHeuristic(s)
         <= mTruncationFactor*(mGraph[s].getCostToCome()+this->getGraphHeuristic(s) ) )
      {
        //std::cout << "T1! Did not expand vertex " << s << std::endl;
        mTruncated.insert(s);
      }
      else
      {
        //std::cout << "Underconsistent! Make " << s << " overconsistent, update neighbors." << std::endl;
        // Otherwise it is underconsistent, no consistent vertices are in the queue
        // Make it overconsistent or consistent
        mGraph[s].setCostToCome(std::numeric_limits<double>::infinity());

        // Update this vertex
        this->updateVertex(s);

        // Now update the sucessor vertices
        NeighborIter ni, ni_end;
        for (boost::tie(ni, ni_end) = adjacent_vertices(s, mGraph); ni != ni_end; ++ni)
          this->updateVertex(*ni);

        }//End else no truncation underconsistent case
    }// End else underconsistent cases

    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    mTotalVertexExpansionTime += time_span.count();

  }// End while

  mPlannerStatus = PlannerStatus::Solved;
  //mQueue.printQueue();
}

// ============================================================================
void TLPAstar::computeGpi(Vertex s) {
    constexpr double inf = std::numeric_limits<double>::infinity();

    double cost = 0.0;
    Vertex v = s;
    std::vector<Vertex> pi;
    std::set<Vertex> visited;

    while(v != mSourceVertex) {
        if (visited.find(v) != visited.end() || !mGraph[v].hasParent()) {
            // std::cout << "computeGpi(" << s << "): Early termination!";
            // for(Vertex u : pi) std::cout << " " << u;
            // std::cout << std::endl;

            mGraph[s].setGpi(inf);
            mGraph[s].removePi();
            return;
        }
        else if (isTruncated(v)) {
            cost += mGraph[v].getGpi();
            for(Vertex u : mGraph[v].getPi()) {
                pi.push_back(u);
            }
            break;
        }
        visited.insert(v);
        Edge edge = this->getEdge(mGraph[v].getParent(), v);
        double incCost = mGraph[edge].getValue();

        cost += incCost;
        pi.push_back(mGraph[v].getParent());
        v = mGraph[v].getParent();
    }
    mGraph[s].setGpi(cost);
    mGraph[s].setPi(pi);

    // Debug output
    // std::cout << "computeGpi(" << s << "): " << cost;
    // for(Vertex v : pi) std::cout << " " << v;
    // std::cout << std::endl;
}

// ============================================================================
Path TLPAstar::obtainPath(Vertex s)
{
  Path pi;
  pi.push_back(s);
  //std::cout << "obtainPath:" << std::endl << s << std::endl;;

  while(s != mSourceVertex)
  {
    if(isTruncated(mGraph[s].getParent()))
    {
      //std::cout << "T" << std::endl;;
      pi.push_back(mGraph[s].getParent());
      //std::cout << mGraph[s].getParent() << std::endl;;
      for(Vertex v : mGraph[mGraph[s].getParent()].getPi()) {
          pi.push_back(v);
          //std::cout << v << std::endl;
      }
      //std::cout << std::endl;
      return pi;
    }
    pi.push_back(mGraph[s].getParent());
    s = mGraph[s].getParent();
    //std::cout << s << std::endl;
  }
  //std::cout << std::endl;
  return pi;
}

// ============================================================================
// This clears the list of truncated vertices by making them overconsistent.
void TLPAstar::clearTruncatedVertices()
{
    // std::cout << "Clearing truncated vertices." << std::endl;
    auto it = mTruncated.begin();
    while(it != mTruncated.end())
    {
        Vertex v = *it;
        it = mTruncated.erase(it);
        mGraph[v].setGpi(std::numeric_limits<double>::infinity());
        mGraph[v].removePi();
        this->updateVertex(v);
    }
}


// ============================================================================
bool TLPAstar::isTruncated(Vertex v) {
    return mTruncated.find(v) != mTruncated.end();
}



// ============================================================================
bool TLPAstar::perceiveChanges() {

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

    this->clearTruncatedVertices();
    return isChanged;
}

// ============================================================================
Edge TLPAstar::getEdge(Vertex u, Vertex v) {
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);
  assert(edgeExists);

  return uv;
}

// ============================================================================
Path TLPAstar::getPath() {
  return this->obtainPath(mTargetVertex);
}


// ============================================================================
// TODO (avk): I should be able to set the heuristic function from the demo
// script. Create a Heuristic Class and send it in. Have a default heuristic
// if nothing has been set.
double TLPAstar::getGraphHeuristic(Vertex v) {
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());
  return heuristic;
}

// ============================================================================
void TLPAstar::setConnectionRadius(double radius) {
  mConnectionRadius = radius;
}

// ============================================================================
double TLPAstar::getConnectionRadius() {
  return mConnectionRadius;
}

// ============================================================================
void TLPAstar::setCollisionCheckResolution(double resolution) {
  mCollisionCheckResolution = resolution;
}

// ============================================================================
double TLPAstar::getCollisionCheckResolution() {
  return mCollisionCheckResolution;
}

// ============================================================================
void TLPAstar::setRoadmap(std::string filename) {
  if (filename == "")
    std::invalid_argument("Roadmap Filename cannot be empty!");

  // Load the graph.
  mRoadmap = boost::shared_ptr<io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>>(
      new io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>(mSpace, filename));

  mRoadmap->generate(
      mGraph, get(&VertexProperties::mState, mGraph), get(&EdgeProperties::mLength, mGraph));

  //// Loop over edges, set length according to state check for consistency.
  // EdgeIter ei, ei_end;
  // for (boost::tie(ei, ei_end) = edges(mGraph); ei != ei_end; ++ei) {
  //     Vertex startVertex = source(*ei, mGraph);
  //     Vertex endVertex = target(*ei, mGraph);
  //     auto startState = mGraph[startVertex].getState()->getOMPLState();
  //     auto endState = mGraph[endVertex].getState()->getOMPLState();
  //     mGraph[*ei].setLength(mSpace->distance(startState, endState));
  // }

  // Mark the graph to have been setup.
  mGraphSetup = true;
}

// ============================================================================
double TLPAstar::distFun(const lgls::datastructures::Vertex& v1, const lgls::datastructures::Vertex& v2) {
  mGraph[v1].getState()->getOMPLState();
  return mSpace->distance(
      mGraph[v1].getState()->getOMPLState(), mGraph[v2].getState()->getOMPLState());
}

// ============================================================================
void TLPAstar::setupKNN() {
  knnGraph.setDistanceFunction(
      std::bind(&TLPAstar::distFun, this, std::placeholders::_1, std::placeholders::_2));
}


// ============================================================================
void TLPAstar::setBestPathCost(double cost) {
  mBestPathCost = cost;
}

// ============================================================================
double TLPAstar::getBestPathCost() {
  return mBestPathCost;
}

// ============================================================================
void TLPAstar::setTruncationFactor(double factor) {
  mTruncationFactor = factor;
  assert(mTruncationFactor >= 1.0);
}

// ============================================================================
double TLPAstar::getTruncationFactor() {
  return mTruncationFactor;
}

// ============================================================================
void TLPAstar::setPlannerStatus(PlannerStatus status) {
  mPlannerStatus = status;
}

// ============================================================================
PlannerStatus TLPAstar::getPlannerStatus() {
  return mPlannerStatus;
}

// ============================================================================
double TLPAstar::getNumberOfEdgeEvaluations() {
  return mNumberOfEdgeEvaluations;
}

// ============================================================================
double TLPAstar::getNumberOfVertexExpansions() {
  return mNumberOfVertexExpansions;
}

// ============================================================================
CollisionStatus TLPAstar::evaluateVertex(const Vertex& v) {
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
CollisionStatus TLPAstar::evaluateEdge(const Edge& e) {

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
    if (evaluateVertex(startVertex)==CollisionStatus::Collision
        || evaluateVertex(endVertex)==CollisionStatus::Collision)
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
ompl::base::PathPtr TLPAstar::constructSolution(const Vertex& source, const Vertex& target) {
  ompl::geometric::PathGeometric* path = new ompl::geometric::PathGeometric(si_);
  Path p = this->obtainPath(target);
  for (Vertex v : p)
    path->append(mGraph[v].getState()->getOMPLState());
  path->reverse();
  return ompl::base::PathPtr(path);
}


// ============================================================================
void TLPAstar::generateNewSamples(int batchSize, bool updateVertices) {

  // Reset counters;
  mNumberOfEdgeEvaluations=0;

  mNumberOfVertexExpansions=0;

  mTotalEdgeEvaluationTime=0;

  mTotalVertexExpansionTime=0;

  // Vertices to be updated if updateVertices is set True
  std::vector<Vertex> verticesTobeUpdated;
  // Collect near samples
  std::vector<Vertex> nearestSamples;

  auto validityChecker = si_->getStateValidityChecker();
  unsigned int dim = si_->getStateDimension();


  const ompl::base::RealVectorBounds &bounds
    = static_cast<const ompl::base::RealVectorStateSpace*>(mSpace.get())->getBounds();


  // Scale to required limits.
  int numSampled = 0;
  while (numSampled < batchSize) {

    // ================= Hatlon Sequence ====================//
    // auto newPosition = mHaltonSequence->sample();
    //
    // for (unsigned int i = 0; i < dim; ++i)
    //   newPosition[i] = bounds.low[i] + newPosition[i] * (bounds.high[i] - bounds.low[i]);
    //
    //
    // // Our ompl::base::State* wrapper
    // StatePtr sampledState(new lgls::datastructures::State(mSpace));
    //
    // // copy the Halton sequenced point to ompl::base::State*
    // mSpace->copyFromReals(sampledState->getOMPLState(), newPosition);

    // ================= Uniform Sampler  ====================//
    // Our ompl::base::State* wrapper
    StatePtr sampledState(new lgls::datastructures::State(mSpace));

    // copy the Halton sequenced point to ompl::base::State*
    mUniformSampler->sampleUniform(sampledState->getOMPLState());
    // mSpace->copyFromReals(sampledState->getOMPLState(), newPosition);


    // ================= Check validity   ====================//
    auto validityChecker = si_->getStateValidityChecker();

    // only for static environment, we don't want to discard possible vertices for dynamic env
    if(!validityChecker->isValid(sampledState->getOMPLState()))
      continue;

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
    // std::cout << "connecting "<<*it << " with: ";
    for (const auto& v : nearestSamples) {
      if(*it==v) continue;
      // std::cout << v << ", ";
      double distance = mSpace->distance(
          mGraph[v].getState()->getOMPLState(), mGraph[*it].getState()->getOMPLState());
      std::pair<Edge, bool> newEdge = boost::add_edge(*it, v, mGraph);
      mGraph[newEdge.first].setLength(distance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
      assert(newEdge.second);
    }
    // std::cout << std::endl;

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

    // pdef_->clearSolutionPaths();
  }

  this->clearTruncatedVertices();
  OMPL_INFORM("A new batch of %d samples generated",batchSize);
}

double TLPAstar::calculateR() const
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
void TLPAstar::call_visualize()
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
// ============================================================================
// void TLPAstar::printEdge(const Edge& edge){
//   ColorStatus color = mGraph[edge].getColorStatus();
//   std::cout << "Edge " << edge << " is ";
//
//   if (color == ColorStatus::Feasible) {
//     std::cout << "Feasible   ";
//   } else if (color == ColorStatus::Infeasible) {
//     std::cout << "Infeasible ";
//   } else {
//     std::cout << "Unknown    ";
//   }
//
//   std::cout << ", Length: " << mGraph[edge].getLength()
//             << ", Value: " << mGraph[edge].getValue()
//             << std::endl;
// }
//
// void TLPAstar::printVertex(Vertex v) {
//     std::cout
//         << "Vertex " << v << ": "
//         << "v = " << mGraph[v].getCostToCome() << "; "
//         << "g = " << mGraph[v].getRHS() << "; "
//         << "h = " << mGraph[v].getHeuristic() << "; "
//         << "gpi = " << mGraph[v].getGpi() << "; ";
//
//     std::cout << "bp = ";
//     if (mGraph[v].hasParent())
//         std::cout << mGraph[v].getParent() << "; ";
//     else
//         std::cout << "null; ";
//
//     std::cout << std::endl;
// }

} // namespace lgls

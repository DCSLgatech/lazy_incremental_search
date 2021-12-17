/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim, Connor Lawson */

#include "lgls/BLGLS.hpp"

#include <cmath>    // pow, sqrt
#include <iostream> // std::invalid_argument
#include <set>      // std::set
#include <assert.h> // debug

#include <Eigen/Geometry>
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

BLGLS::BLGLS(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::Planner(si, "BLGLS"), mSpace(si->getStateSpace()){
  // Set default values for data members.
  // Halton sequence sampler for n dimensional search
  mHaltonSequence = std::make_shared<ompl::base::HaltonSequence>(si->getStateDimension());

  // Uniform sampler
  mUniformSampler = si->allocStateSampler();
}

BLGLS::~BLGLS() {
  // Do nothing.
}

// ============================================================================
void BLGLS::setup() {
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
void BLGLS::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) {
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
void BLGLS::setupPreliminaries() {

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

  // use knn instead of R-disc to ensure start/goal are connected sufficiently
  knnGraph.nearestK(mSourceVertex, mKNeighbors, nearestSource);
  // Add nearest vertices around the source to the graph
  // knnGraph.nearestR(mSourceVertex, mConnectionRadius, nearestSource);

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
  knnGraph.nearestK(mTargetVertex, mGoalKNeighbors, nearestTarget);
  // knnGraph.nearestR(mTargetVertex, mConnectionRadius, nearestTarget);

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

  // Setup the event.
  mEvent->setup(&mGraph, mSourceVertex, mTargetVertex);

}

// ============================================================================
void BLGLS::freshStart() {

  // Call the base clear
  ompl::base::Planner::clear();

  // Clear the queues.
  mQueue.clear();
  assert(mQueue.isEmpty());

  mTruncated.clear();
  assert(mTruncated.empty());

  knnGraph.clear();
  mGraph.clear();
  mNumberOfEdgeEvaluations = 0;
  mNumberOfVertexExpansions = 0;
  mPlannerStatus = PlannerStatus::NotSolved;
  OMPL_INFORM("Removed Everything");
}


// ============================================================================
// clear search, leave graph, remove start and goal vertices.
void BLGLS::clear() {
  // Call the base clear
  ompl::base::Planner::clear();

  // Clear the queues.
  mQueue.clear();
  assert(mQueue.isEmpty());

  mTruncated.clear();
  assert(mTruncated.empty());

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

  // Remove from knn the start the the goal
  knnGraph.remove(mSourceVertex);
  knnGraph.remove(mTargetVertex);

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
void BLGLS::setDebugCallback(std::function<void(Graph)> callback) {
    mCallback = callback;
}

// ============================================================================
ompl::base::PlannerStatus BLGLS::solve(const ompl::base::PlannerTerminationCondition& ptc) {
  // TODO (avk): Use ptc to terminate the search.


  // Visualization thread clock
  // mJoined = false;
  // std::thread visualize_thread{[this]() {call_visualize(); }};

  mPreempt = false;
  mPlannerStatus = PlannerStatus::NotSolved;

  /// Let's see if source or target are in collision
  if (this->evaluateVertex(mSourceVertex) == CollisionStatus::Collision) {
    OMPL_INFORM("Start State is invalid.");
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (this->evaluateVertex(mTargetVertex) == CollisionStatus::Collision) {
    OMPL_INFORM("Goal State is invalid.");
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  // Sample first a batch.
  this->generateNewSamples(mSampleMultiplier, mSampleBufferSize, false);

  // Initialize the search
  this->updateVertex(mSourceVertex);

  while (!ptc && !mPreempt)
  {
    // Run in loop.
    // while (mPlannerStatus != PlannerStatus::Solved) {
    while(true) {
      // // Claim the mutex to use graph
      // std::unique_lock<std::mutex> lck{mtx};
      //
      // // Wait until finish drawing the graph
      // cv.wait(lck);

      /// Repair the tree till the event is triggered. returns the leaf
      Vertex triggeredLeaf;
      if(this->computeShortestPath(triggeredLeaf))
      {

        // std::cout << "Lazy LPA* returns a path "<< triggeredLeaf << std::endl;
        /// Evaluate along the subpath to the leaf, returns the first inconsis edge
        Edge inconsistentEdge ;
        bool inconsistencyExist = this->evaluatePath(triggeredLeaf,inconsistentEdge);

        // std::cout << "  evauated this path "<< triggeredLeaf << std::endl;
        // if(mCallback) mCallback(mGraph);
        /// Let the lazy LPA* handle the inconsistency
        if (inconsistencyExist) {
          // Update vertices around inconsistent edge
          Vertex u = source(inconsistentEdge, mGraph);
          Vertex v = target(inconsistentEdge, mGraph);
          this->updateVertex(u);
          this->updateVertex(v);
          // this->updateVertex(target(inconsistentEdge, mGraph));
          // if(mCallback) mCallback(mGraph);
          this->clearTruncatedVertices();
          // std::cout<< "inconsistent edge found updated" <<std::endl;
        }
        else if (triggeredLeaf == mTargetVertex)
        {
          // if(mCallback) mCallback(mGraph);

          // NO inconsistent edge is found,
          // if the triggering vertex is not a goal, we need to keep growing
          // but updating triggeredLeaf won't do anything to this vertex,
          // since it is already consistent, Hence, propagate forward.
            OMPL_INFORM("No inconsistent edge found. Solved!");
            mPlannerStatus = PlannerStatus::Solved;
            break;
        }

      } else
      {
        // No triggering vertex exists
        OMPL_INFORM("No Trigerring Vertex Exists in the graph");
        // std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        //   << ", Edge Evaluated: "<< mNumberOfEdgeEvaluations
        //   << ", Queue Size: " <<  mQueue.getSize() << std::endl;
        break;
      }

    } // End while inner-loop

    // Add an additional batch of samples, and updates these newly added ones.
    this->generateNewSamples(mSampleMultiplier, mSampleBufferSize, true);

  }  // End while outer-loop


  // Visualization clock thread joined
  // mJoined = true;
  // visualize_thread.join();

  // Report the timing average results
  // std::cout << "Average Time to evaluate an edge : " << mTotalEdgeEvaluationTime/mNumberOfEdgeEvaluations << " s" <<std::endl;
  // std::cout << "Average Time to exapnd a vertex : " << mTotalVertexExpansionTime/mNumberOfVertexExpansions << " s" <<std::endl;


  // if (mPlannerStatus == PlannerStatus::Solved && !std::isinf(mGraph[mTargetVertex].getCostToCome())) {
  if (mPlannerStatus == PlannerStatus::Solved) {
    this->computeGpi(mTargetVertex);
    this->setBestPathCost(mGraph[mTargetVertex].getGpi());
    pdef_->addSolutionPath(constructSolution(mSourceVertex, mTargetVertex));

    OMPL_INFORM("Plan Found.");
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }
  if (mPreempt) {
    OMPL_INFORM("Planning Aborted.");
    return ompl::base::PlannerStatus::ABORT;
  }
  OMPL_INFORM("Planning TIMEOUT.");
  return ompl::base::PlannerStatus::TIMEOUT;
}

// ============================================================================
Keys BLGLS::calculateKeys(Vertex u) {
  double minval = std::min(mGraph[u].getCostToCome(),mGraph[u].getRHS());
  return std::make_pair(minval+this->getGraphHeuristic(u), minval);

}

// ============================================================================
void BLGLS::updateVertex(Vertex v) {
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

      if (mGraph[u].getCostToCome() + mGraph[uv].getValue(mInflationFactor) < tempRHS)
      {
            tempRHS = mGraph[u].getCostToCome() + mGraph[uv].getValue(mInflationFactor);
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
bool BLGLS::computeShortestPath(Vertex& triggeredLeafVertex) {

  while (mQueue.keyComparison(mQueue.getTopVertexKeys(), this->calculateKeys(mTargetVertex)) ||
  mGraph[mTargetVertex].getRHS() > mGraph[mTargetVertex].getCostToCome() ) {

    // For visualization only, safe to comment out or remove if not needed.
    // if(mCallback) mCallback(mGraph);

    auto tic = std::chrono::high_resolution_clock::now();

    if (mQueue.isEmpty()){
      OMPL_INFORM("Queue is emptied");
      std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        << ", Edge Evaluated: "<< mNumberOfEdgeEvaluations
        << ", Queue Size: " <<  mQueue.getSize() << std::endl;

      return false;
    }
    // Pop front vertex from the queue
    // Vertex s = mQueue.popTopVertex();
    Vertex s = mQueue.getTopVertex();

    // // Count the number of expansion
    // mNumberOfVertexExpansions++;

    // std::cout << "poped, compute Gpi of target now...: " << std::endl;
    // Compute Gpi of the goal vertex
    this->computeGpi(mTargetVertex);

    // Second truncation rule check
    // -- is the current path length already bounded subopitmal?
    // std::cout << "T2: " << std::endl;
    if (mGraph[mTargetVertex].getGpi() <= mTruncationFactor
          *(std::min(mGraph[s].getCostToCome(),mGraph[s].getRHS())+this->getGraphHeuristic(s) ) )
    {
      // Yes, expanding the current minimum key vertex has not so much benefit
      triggeredLeafVertex = mTargetVertex;

      // std::cout << "T2! TLPA* terminated before expanding vertex " << s << std::endl;

      // Vertex Expansion timer off before exit
      auto toc = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
      mTotalVertexExpansionTime += time_span.count();
      return true;
    }

    mQueue.removeVertex(s);

    // Is it overconsistent?
    if (mGraph[s].getCostToCome() >mGraph[s].getRHS() ) {

      // Yes, Make it consistent
      mGraph[s].setCostToCome(mGraph[s].getRHS());

      // Now update the sucessor vertices
      NeighborIter ni, ni_end;
      for (boost::tie(ni, ni_end) = adjacent_vertices(s, mGraph); ni != ni_end; ++ni) {
        Vertex u = *ni;
        this->updateVertex(u);
      } // End for successor vertices

      // Count the number of expansion
      mNumberOfVertexExpansions++;
      // After making it consistent, check if this triggers event (LPA* Thm 6)
      // if (mEvent->isTriggered(s)) {

      // TODO: Make sure ConstantDepthEvent checks along the current best path.
      if(mGraph[s].getGpi() < std::numeric_limits<double>::infinity()
          && mEvent->isTriggered(obtainPath(s)) ) {
        triggeredLeafVertex = s;

        // Vertex Expansion timer off before exit
        auto toc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
        mTotalVertexExpansionTime += time_span.count();

        return true;
      }// No else, continue repairing.


    }
    else {
      // Otherwise it is underconsistent, since no consistent vertices are in the queue (TLPA* Lemma2)

      // Compute ObtainPath of s
      this->computeGpi(s);
      // std::cout << "T1: "<< std::endl;
      // First truncation rule check for underconsistent vertex
      if(mGraph[s].getGpi() + this->getGraphHeuristic(s)
         <= mTruncationFactor*(mGraph[s].getCostToCome()+this->getGraphHeuristic(s) ) )
      {
          mTruncated.insert(s);
          // We don't check event triggering for underconsistent vertices, continue

          // std::cout << "T1! Did not expand vertex " << s << std::endl;
      }
      else
      {
        // Cannot truncate this underconsistent vertex,
        // Let's make it overconsistent (or consistent when rhs=inf)
        mGraph[s].setCostToCome(std::numeric_limits<double>::infinity());

        // Update this vertex
        this->updateVertex(s);

        // Now update the sucessor vertices
        NeighborIter ni, ni_end;
        for (boost::tie(ni, ni_end) = adjacent_vertices(s, mGraph); ni != ni_end; ++ni) {
          Vertex u = *ni;
          this->updateVertex(u);
        } // End for successor vertices

        // Count the number of expansion
        mNumberOfVertexExpansions++;
      } // End else no truncation underconsistent case

    } // End else underconsistent cases

    // Vertex Expansion timer set off
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic);
    mTotalVertexExpansionTime += time_span.count();

  } // End while loop

  // std::cout << "compute shortest path while loop escape" <<std::endl;
  // Okay, the tree seems to be consistent all the way up to the goal.
  // Let's return the goal if it is connected
  triggeredLeafVertex = mTargetVertex;
  this->computeGpi(triggeredLeafVertex);

  if (mGraph[triggeredLeafVertex].getGpi()<std::numeric_limits<double>::infinity())
  {
      return true;
  }


  // Otherwise, the goal is not in connected component
  return false;

}


// ============================================================================
void BLGLS::computeGpi(Vertex s) {
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
        // getValue(mInflationFactor) will return
        // w_bar = { mInflationFactor*w_hat, if not evaluated,
        //         { w,                      otherwise.
        double incCost = mGraph[edge].getValue(mInflationFactor);

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
Path BLGLS::obtainPath(Vertex s)
{
  Path pi;
  pi.push_back(s);
  //std::cout << "obtainPath:" << std::endl << s << std::endl;;

  while(s != mSourceVertex)
  {
    // assert(mGraph[s].hasParent());
    if(isTruncated(mGraph[s].getParent()))
    {
      //std::cout << "T" << std::endl;;
      pi.push_back(mGraph[s].getParent());        // getPi does not contain the vertex itself
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
void BLGLS::clearTruncatedVertices()
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
bool BLGLS::isTruncated(Vertex v) {
    return mTruncated.find(v) != mTruncated.end();
}


// ============================================================================
bool BLGLS::evaluatePath(const Vertex& triggeredLeaf, Edge& inconsistentEdge) {

  Path path2Evaluate = this->obtainPath(triggeredLeaf);

  // std::cout << "evaluating obtainPath:";
  // for(Vertex v : path2Evaluate) std::cout << " " << v;
  // std::cout << std::endl;

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
      double oldVal = mGraph[edgeToEvaluate].getValue(mInflationFactor);
      // Now is the good time to evaluate
      this->evaluateEdge(edgeToEvaluate);

      // Has it been changed?
      if ( mGraph[edgeToEvaluate].getValue(mInflationFactor) != oldVal )
      {
        // std::cout << "Edge inconsistent, return to main loop." << std::endl;
        inconsistentEdge =  edgeToEvaluate;
        return true;
      }
      // No else, continue to the next unevaluated edge
      // std::cout << "Evaluated to be consistent move on." << std::endl;
    } // End If not evaluated edge

  } // End For path iteration
  // std::cout << "All edges are consistent." << std::endl;
  // Nay, nothing special
  return false;

}

// ============================================================================
void BLGLS::perceiveNewWorld() {
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    // This is for identifying changed vertices.... only for approximate change detection
    this->evaluateVertex(*vi);
  } // End For vertex iteration

  OMPL_INFORM("First time world is seen");
}


bool BLGLS::perceiveChanges() {

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
  ///    - Check the adjacent edges that have been evaluated.               ///
  ///    - Given that either one end vertex changed the CollisionStatus,    ///
  ///      the edge could have been changed.                                ///
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
  /////////////////////////////// Vertex Approximate Evaluation /////////////////
  // VertexIter vi, vi_end;
  // for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
  //
  //   ColorStatus oldStartColor = mGraph[*vi].getColorStatus();
  //   if (this->evaluateVertex(*vi) != oldStartColor )
  //   {
  //     // Oops change! Collect all incident edges
  //     NeighborIter ni, ni_end;
  //     for (boost::tie(ni, ni_end) = adjacent_vertices(*vi, mGraph); ni != ni_end; ++ni) {
  //       Vertex u = *ni;
  //       // Get the edge between the two vertices.
  //       Edge edge = this->getEdge(u, *vi);
  //       // No need to insert already unknown edge
  //       if (mGraph[edge].getColorStatus() !=ColorStatus::Unknown)
  //       {
  //         mGraph[edge].setColorStatus(ColorStatus::Unknown);
  //         mGraph[edge].setEvaluationStatus(EvaluationStatus::NotEvaluated);
  //         mGraph[edge].setValue(mGraph[edge].getLength());
  //         mPerceivedChangedEdges.push_back(edge);
  //       }
  //
  //
  //     }//End For neighboring edges
  //   } //End If vertex change
  // } // End For vertex iteration
  // std::cout << "Perceives "<< mPerceivedChangedEdges.size() <<" edges change" << std::endl;
  // if (!mPerceivedChangedEdges.empty()){
  //   for (std::vector<Edge>::iterator it = mPerceivedChangedEdges.begin() ; it != mPerceivedChangedEdges.end(); ++it) {
  //     Vertex startVertex = source(*it, mGraph);
  //     Vertex endVertex = target(*it, mGraph);
  //
  //     // Important : Make sure if the source Vertex is changed, then make it inconsistent
  //     if (startVertex == mSourceVertex) mGraph[startVertex].setCostToCome(std::numeric_limits<double>::infinity());
  //
  //     this->updateVertex(startVertex);
  //     this->updateVertex(endVertex);
  //
  //
  //   }
  //   // Okay, there is some change, we should re-solve it.
  //   isChanged = true;
  //   mPlannerStatus = PlannerStatus::NotSolved;
  //   pdef_->clearSolutionPaths();
  // }

 // return isChanged;
}


// ============================================================================
void BLGLS::setEvent(EventPtr event) {
  mEvent = event;
}

// ============================================================================
ConstEventPtr BLGLS::getEvent() const {
  return mEvent;
}

// ============================================================================
Edge BLGLS::getEdge(Vertex u, Vertex v) {
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);
  assert(edgeExists);

  return uv;
}

// ============================================================================
Path BLGLS::getPath() {
  return this->obtainPath(mTargetVertex);
}

// Depreciated
// ============================================================================
Path BLGLS::getPathToSource(Vertex u) {
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
double BLGLS::getGraphHeuristic(Vertex v) {
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());
  return heuristic;
}

// ============================================================================
// void BLGLS::setSpaceInformation(const ompl::base::SpaceInformationPtr& si) {
//   si_ = si;
//   mSpace = si->getStateSpace();
// }

// ============================================================================
void BLGLS::setKNeighbors(int num_neighbors) {
  mKNeighbors = num_neighbors;
}

// ============================================================================
void BLGLS::setGoalKNeighbors(int num_neighbors) {
  mGoalKNeighbors = num_neighbors;
}

// ============================================================================
int BLGLS::getKNeighbors() {
  return mKNeighbors;
}

// ============================================================================
int BLGLS::getGoalKNeighbors() {
  return mGoalKNeighbors;
}

// ============================================================================
void BLGLS::setSampleMultiplier(double sample_mul) {
  mSampleMultiplier = sample_mul;
}

// ============================================================================
double BLGLS::getSampleMultiplier() {
  return mSampleMultiplier;
}

// ============================================================================
void BLGLS::setSampleBufferSize(double buffer) {
  mSampleBufferSize = buffer;
}

// ============================================================================
double BLGLS::getSampleBufferSize() {
  return mSampleBufferSize;
}
// TODO: depreciate this, since we are using calculateR func
// ============================================================================
void BLGLS::setConnectionRadius(double radius) {
  mConnectionRadius = radius;
}

// ============================================================================
double BLGLS::getConnectionRadius() {
  return mConnectionRadius;
}

// ============================================================================
void BLGLS::setCollisionCheckResolution(double resolution) {
  mCollisionCheckResolution = resolution;
}

// ============================================================================
double BLGLS::getCollisionCheckResolution() {
  return mCollisionCheckResolution;
}

// ============================================================================
void BLGLS::setRoadmap(std::string filename) {
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
double BLGLS::distFun(const lgls::datastructures::Vertex& v1, const lgls::datastructures::Vertex& v2) {
  mGraph[v1].getState()->getOMPLState();
  return mSpace->distance(
      mGraph[v1].getState()->getOMPLState(), mGraph[v2].getState()->getOMPLState());
}

// ============================================================================
void BLGLS::setupKNN() {
  knnGraph.setDistanceFunction(
      std::bind(&BLGLS::distFun, this, std::placeholders::_1, std::placeholders::_2));
}


// ============================================================================
void BLGLS::setBestPathCost(double cost) {
  mBestPathCost = cost;
}

// ============================================================================
double BLGLS::getBestPathCost() {
  return mBestPathCost;
}

// ============================================================================
void BLGLS::setInflationFactor(double factor) {
  mInflationFactor = factor;
}

// ============================================================================
double BLGLS::getInflationFactor() {
  return mInflationFactor;
}

// ============================================================================
void BLGLS::setTruncationFactor(double factor) {
  mTruncationFactor = factor;
}

// ============================================================================
double BLGLS::getTruncationFactor() {
  return mTruncationFactor;
}


// ============================================================================
void BLGLS::setPlannerStatus(PlannerStatus status) {
  mPlannerStatus = status;
}

// ============================================================================
PlannerStatus BLGLS::getPlannerStatus() {
  return mPlannerStatus;
}

// ============================================================================
double BLGLS::getNumberOfEdgeEvaluations() {
  return mNumberOfEdgeEvaluations;
}

// ============================================================================
double BLGLS::getNumberOfVertexExpansions() {
  return mNumberOfVertexExpansions;
}

// ============================================================================
CollisionStatus BLGLS::evaluateVertex(const Vertex& v) {
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
CollisionStatus BLGLS::evaluateEdge(const Edge& e) {

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

  if(!getSpaceInformation()->checkMotion(startState, endState))
  {
    // Okay, this motion is not valid (checkMotion uses si stateValidityChecker)
    // Set value to infinty
    edgeColor =  CollisionStatus::Collision;
  }

  // // Proceed to the intermideate states, if passed the start and end vertices check
  // if (edgeColor != CollisionStatus::Collision)
  // {
  //   // Access the validity checker.
  //   auto validityChecker = si_->getStateValidityChecker();
  //
  //   // Evaluate the state in between.
  //   int maxSteps = 1.0 / mCollisionCheckResolution;
  //   for (int multiplier = 1; multiplier < maxSteps + 1; ++multiplier) {
  //     double interpolationStep = mCollisionCheckResolution * multiplier;
  //     assert(interpolationStep <= 1);
  //     StatePtr midVertex(new lgls::datastructures::State(mSpace));
  //     mSpace->interpolate(startState, endState, interpolationStep, midVertex->getOMPLState());
  //
  //     if (!validityChecker->isValid(midVertex->getOMPLState())){
  //       edgeColor = CollisionStatus::Collision;
  //       break; // No need to check further
  //     }
  //   } // End For interpolation
  // } // End If passed start-end check


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
ompl::base::PathPtr BLGLS::constructSolution(const Vertex& source, const Vertex& target) {
  // ompl::geometric::PathGeometric* path = new ompl::geometric::PathGeometric(si_);
  // Vertex v = target;
  // // std::cout << "path : " ;
  // while (v != source) {
  //   // std::cout << v << ", " ;
  //   path->append(mGraph[v].getState()->getOMPLState());
  //   v = mGraph[v].getParent();
  // }
  // // std::cout << std::endl;
  // if (v == source) {
  //   path->append(mGraph[source].getState()->getOMPLState());
  // }
  // path->reverse();
  // return ompl::base::PathPtr(path);
  ompl::geometric::PathGeometric* path = new ompl::geometric::PathGeometric(si_);
  Path p = this->obtainPath(target);
  for (Vertex v : p)
    path->append(mGraph[v].getState()->getOMPLState());
  path->reverse();
  return ompl::base::PathPtr(path);
}


// ============================================================================
std::vector<double> BLGLS::haltonSample(std::size_t index) const {
  std::vector<int> bases{2, 3};

  // Generate a new sample.
  std::vector<double> sample;
  for (const auto& base : bases) {
    auto tempIndex = index;
    double result = 0.0;
    double f = 1.0;
    while (tempIndex > 0) {
      f /= base;
      result += f * (tempIndex % base);
      tempIndex /= base;
    }
    sample.push_back(result);
  }
  return sample;

}


// ============================================================================
void BLGLS::generateNewSamples(int batchSize, bool updateVertices) {

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

    // Informed sampling
    double g_hat = mSpace->distance(
        mGraph[mSourceVertex].getState()->getOMPLState(), sampledState->getOMPLState());

    double h_hat = mSpace->distance(
        sampledState->getOMPLState(), mGraph[mTargetVertex].getState()->getOMPLState());

    if (getBestPathCost() < g_hat+h_hat)
      continue;

    // ================= Check validity   ====================//
    auto validityChecker = si_->getStateValidityChecker();

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

// ============================================================================
void BLGLS::generateNewSamples(double sample_multiplier, double buffer, bool updateVertices) {

  std::vector<double> sourcePosition;
  std::vector<double> targetPosition;
  mSpace->copyToReals(sourcePosition, mGraph[mSourceVertex].getState()->getOMPLState());
  mSpace->copyToReals(targetPosition, mGraph[mTargetVertex].getState()->getOMPLState());

  // Find euc dist but cut off w
  Eigen::Vector2d sourceVect = Eigen::Vector2d(sourcePosition.data());
  Eigen::Vector2d targetVect = Eigen::Vector2d(targetPosition.data());

  double euc_dist = this->getGraphHeuristic(mSourceVertex);

  // Setup
  Eigen::Rotation2D<double> rot90Clock(-M_PI / 2);
  Eigen::Rotation2D<double> rot90CounterClock(M_PI / 2);

  // Define origin x_axis/y_axis vectors for sampling points in rectangle
  Eigen::Vector2d buffSource = (1 + buffer / euc_dist) * (sourceVect - targetVect) + targetVect;
  Eigen::Vector2d x_axis = (1 + 2 * buffer / euc_dist) * (targetVect - sourceVect);
  Eigen::Vector2d y_axis = rot90CounterClock.toRotationMatrix() * x_axis;
  Eigen::Vector2d bottom_v = rot90Clock.toRotationMatrix() * x_axis;
  Eigen::Vector2d origin = buffSource + 0.5 * bottom_v;
  double zmin = -M_PI;
  double zmax = M_PI;

  // Sample points inside the space.
  int minBatchSize = 100;
  int batchSize = std::floor(sample_multiplier * euc_dist) > minBatchSize ? std::floor(sample_multiplier * euc_dist) : minBatchSize;

  StatePtr sampledState(new lgls::datastructures::State(mSpace));

  auto validityChecker = si_->getStateValidityChecker();

  // Vertices to be updated if updateVertices is set True
  std::vector<Vertex> verticesTobeUpdated;


  // Scale to required limits.
  int numSampled = 0;
  // mOnlineVertices.reserve(mOnlineVertices.size() + batchSize);
  while (numSampled < batchSize) {

    // assert ReedsShepp
    std::vector<double> newPosition = mHaltonSequence->sample();

    // Scale the halton sample to between the limits.
    Eigen::Vector2d nPosition = Eigen::Vector2d(newPosition.data());
    nPosition = origin + nPosition[0] * x_axis + nPosition[1] * y_axis;
    newPosition = std::vector<double>{
        &nPosition[0], nPosition.data() + nPosition.cols() * nPosition.rows()};

    if (newPosition.size()>2)
      newPosition[2] = zmin + (zmax - zmin) * newPosition[2];

    mSpace->copyFromReals(sampledState->getOMPLState(), newPosition);

    // ================= Check validity   ====================//
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

    // 3. Connect edges
    std::vector<Vertex> nearestSamples;
    // knnGraph.nearestR(sampleVertex, mConnectionRadius, nearestSamples);
    knnGraph.nearestK(sampleVertex, mKNeighbors, nearestSamples);
    // std::cout << "Found " << nearestSamples.size() << "neighors" <<std::endl;
    for (const auto& v : nearestSamples) {
      // No need to assign distance, we are not using any edge heuristic.

      std::pair<Edge, bool> newEdge = boost::add_edge(sampleVertex, v, mGraph);
      double distance = mSpace->distance(
          mGraph[v].getState()->getOMPLState(), mGraph[sampleVertex].getState()->getOMPLState());
      mGraph[newEdge.first].setLength(distance);
      mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);

      assert(newEdge.second);
    }

    knnGraph.add(sampleVertex);
    verticesTobeUpdated.push_back(sampleVertex);
  }

  // Update newly added vertices
  if (updateVertices)
  {
    // Now update the vertices
    for (std::vector<Vertex>::iterator it = verticesTobeUpdated.begin() ; it != verticesTobeUpdated.end(); ++it) {
      this->updateVertex(*it);
    }
    // Okay, there is some change, we should re-solve it.
    // mPlannerStatus = PlannerStatus::NotSolved;
    //
    // pdef_->clearSolutionPaths();
  }

  this->clearTruncatedVertices();
  OMPL_INFORM("Added %d %d samples", numSampled, getSpaceInformation()->getStateDimension());
}

// ============================================================================
double BLGLS::calculateR() const
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
void BLGLS::abortPlanning() {
  mPreempt = true;
}

// ============================================================================
void BLGLS::call_visualize()
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

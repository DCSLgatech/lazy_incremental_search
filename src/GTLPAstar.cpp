/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim, Connor Lawson */

#include "lgls/GTLPAstar.hpp"

#include <cmath>    // pow, sqrt
#include <iostream> // std::invalid_argument
#include <set>      // std::set
#include <assert.h> // debug

#include <boost/graph/connected_components.hpp> // connected_components

using lgls::datastructures::Keys;
using lgls::datastructures::ColorStatus;
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
using lgls::datastructures::VisitStatus;
using lgls::datastructures::VPStateMap;
using lgls::event::ConstEventPtr;
using lgls::event::EventPtr;



namespace lgls {

GTLPAstar::GTLPAstar(const ompl::base::SpaceInformationPtr& si)
  : ompl::base::Planner(si, "GTLPAstar"), mSpace(si->getStateSpace()) {
  // Set default values for data members.
}

GTLPAstar::~GTLPAstar() {
  // Do nothing.
}

// ============================================================================
void GTLPAstar::setup() {
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
void GTLPAstar::setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) {
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
void GTLPAstar::setupPreliminaries() {
  // Issue a warning if mConnectionRadius = 0.

  if(mCallback) mCallback(mGraph);

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

  std::cout << "mSourceVertex: "; printVertex(mSourceVertex);
  std::cout << "mTargetVertex: "; printVertex(mTargetVertex);

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

  // TODO (AVK): Make this kNN + R-disc. Additionally join the start and goal.
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    double sourceDistance
        = mSpace->distance(mGraph[*vi].getState()->getOMPLState(), sourceState->getOMPLState());
    double targetDistance
        = mSpace->distance(mGraph[*vi].getState()->getOMPLState(), targetState->getOMPLState());

    // Cache heuristic value!
    mGraph[*vi].setHeuristic(targetDistance);

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

  mEvent->setup(&mGraph, mSourceVertex, mTargetVertex);

  if(mCallback) mCallback(mGraph);

  // Additionally connect the source and target with a straight line to snap.
  // std::pair<Edge, bool> newEdge = boost::add_edge(mSourceVertex, mTargetVertex, mGraph);
  // mGraph[newEdge.first].setLength(
  //     mSpace->distance(sourceState->getOMPLState(), targetState->getOMPLState()));
  // mGraph[newEdge.first].setEvaluationStatus(EvaluationStatus::NotEvaluated);
  // mGraph[newEdge.first].setCollisionStatus(CollisionStatus::Free);

  // Initialize the search
  mQueue.addVertexWithKeys(mSourceVertex, this->calculateKeys(mSourceVertex));
  mEvent->updateVertexProperties(mSourceVertex);
  if(mCallback) mCallback(mGraph);
}

// ============================================================================
void GTLPAstar::clear() {
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
    mGraph[*vi].setGpi(std::numeric_limits<double>::infinity());
    mGraph[*vi].setColorStatus(ColorStatus::Unknown);
  }

  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(mGraph); ei != ei_end; ++ei) {
    mGraph[*ei].setEvaluationStatus(EvaluationStatus::NotEvaluated);
    mGraph[*ei].setColorStatus(ColorStatus::Unknown);
  }

  // Remove edges between source, target to other vertices.
  clear_vertex(mSourceVertex, mGraph);
  clear_vertex(mTargetVertex, mGraph);

  // Remove the vertices themselves.
  remove_vertex(mSourceVertex, mGraph);
  remove_vertex(mTargetVertex, mGraph);

  setBestPathCost(0);
  mNumberOfEdgeEvaluations = 0;
  mNumberOfEdgeRewires = 0;
  mNumberOfVertexExpansions = 0;
  mTreeValidityStatus = TreeValidityStatus::Valid;
  mPlannerStatus = PlannerStatus::NotSolved;


  OMPL_INFORM("Cleared Everything");
}

// ============================================================================
bool GTLPAstar::evaluatePath(const Vertex& triggeredLeaf, Edge& inconsistentEdge) {
    Path path2Evaluate = this->obtainPath(triggeredLeaf);
    // std::cout << "obtainPath:";
    // for(Vertex v : path2Evaluate) std::cout << " " << v;
    // std::cout << std::endl;

    Edge edgeToEvaluate;
    // Return the first unevaluated edge closest to source.
    for (std::size_t i = path2Evaluate.size() - 1; i > 0; --i) {
        bool edgeExists;
        boost::tie(edgeToEvaluate, edgeExists) = edge(path2Evaluate[i], path2Evaluate[i-1], mGraph);

        // std::cout << "Testing edge: " << edgeToEvaluate << " (" << edgeExists << ")" << std::endl;
        // printEdge(edgeToEvaluate);

        if (mGraph[edgeToEvaluate].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
        {
            // std::cout << "Invalid! Evaluate now." << std::endl;
            // Now is the good time to evaluate
            double oldValue = mGraph[edgeToEvaluate].getValue();
            this->evaluateEdge(edgeToEvaluate);
            //printEdge(edgeToEvaluate);

            // Return this evaluated edge if it is inconsistent to the previous value.
            if (mGraph[edgeToEvaluate].getValue()!= oldValue) {
                // std::cout << "Edge inconsistent, return to main loop." << std::endl;
                inconsistentEdge =  edgeToEvaluate;
                return true;
            }
        }
    }
    return false;
}

// ============================================================================
// This clears the list of truncated vertices by making them overconsistent.
void GTLPAstar::clearTruncatedVertices()
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

void GTLPAstar::setDebugCallback(std::function<void(Graph)> callback) {
    mCallback = callback;
}

// ============================================================================
ompl::base::PlannerStatus GTLPAstar::solve(const ompl::base::PlannerTerminationCondition& /*ptc*/) {
  // TODO (avk): Use ptc to terminate the search.

  /// Let's see if source or target are in collision
  if (this->evaluateVertex(mSourceVertex) == ColorStatus::Infeasible) {
    OMPL_INFORM("Start State is invalid.");
    return ompl::base::PlannerStatus::INVALID_START;
  }

  if (this->evaluateVertex(mTargetVertex) == ColorStatus::Infeasible) {
    OMPL_INFORM("Goal State is invalid.");
    return ompl::base::PlannerStatus::INVALID_GOAL;
  }

  // Main search
  while (mPlannerStatus != PlannerStatus::Solved) {
    if(mCallback) mCallback(mGraph);
    /// Solve current graph using TLPA* search
    Vertex triggeredLeaf;
    if (this->computeShortestPath(triggeredLeaf)) {
        // If true, we hit the event

        /// Evaluate along the path to the target, returns the first inconsistent edge
        Edge inconsistentEdge;
        bool inconsistencyExist = this->evaluatePath(triggeredLeaf,inconsistentEdge);
        //OMPL_INFORM("Finished evaluatePath.");
        if(mCallback) mCallback(mGraph);

        /// Let TLPA* handle the inconsistency as a graph modification.
        if (inconsistencyExist) {
            //OMPL_INFORM("Found inconsistent edge!");
            //printEdge(inconsistentEdge);

            // Update vertices around inconsistent edge
            Vertex u = source(inconsistentEdge, mGraph);
            Vertex v = target(inconsistentEdge, mGraph);
            this->updateVertex(u);
            this->updateVertex(v);

            if(mCallback) mCallback(mGraph);

            // Reset truncated edges before further processing
            // std::cout << "Truncated vertices:";
            // for(Vertex v : mTruncated) std::cout << " " << v;
            // std::cout << std::endl;

            this->clearTruncatedVertices();
        }
        else if (triggeredLeaf == mTargetVertex) {
            OMPL_INFORM("No inconsistent edge found. Solved!");
            mPlannerStatus = PlannerStatus::Solved;
        }
    } else {
        // Only way we return false is by running the queue empty.
        // No triggering vertex exists
        OMPL_INFORM("No Path Exists in the graph");
        std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
            << ", Edge Evaluated: "<< mNumberOfEdgeEvaluations
            << ", Queue Size: " <<  mQueue.getSize() << std::endl;
        break;
    }

  }

  if (mPlannerStatus == PlannerStatus::Solved) {
    this->computeGpi(mTargetVertex);
    this->setBestPathCost(mGraph[mTargetVertex].getGpi());
    pdef_->addSolutionPath(this->constructSolution(mSourceVertex, mTargetVertex));
    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  } else {
    OMPL_INFORM("No Solution Found.");
    return ompl::base::PlannerStatus::TIMEOUT;
  }
}

// ============================================================================
Keys GTLPAstar::calculateKeys(Vertex u) {
  double minval = std::min(mGraph[u].getCostToCome(),mGraph[u].getRHS());
  return std::make_pair(minval+this->getGraphHeuristic(u), minval);

  // TODO: (jil) Do we need to store vertex costToGo heurstic?
  // return std::make_pair(minval+mGraph[u].getHeuristic(), minval);
}

// ============================================================================
bool GTLPAstar::isTruncated(Vertex v) {
    return mTruncated.find(v) != mTruncated.end();
}

// ============================================================================
// Alg3: UpdateState (?)
void GTLPAstar::updateVertex(Vertex v) {
    //std::cout << "Update "; printVertex(v);

    // Do nothing if this is the start vertex
    if (v == mSourceVertex) {
        //std::cout << "Source vertex, do nothing." << std::endl;
        return;
    }

    // Find optimal parent and cost to that parent
    Vertex newParent = boost::graph_traits<lgls::datastructures::BasicGraph>::null_vertex();
    double tempRHS =  std::numeric_limits<double>::infinity();

    // Iterate over all neighbors
    NeighborIter ni, ni_end;
    for (boost::tie(ni, ni_end) = adjacent_vertices(v, mGraph); ni != ni_end; ++ni) {
        Vertex u = *ni;
        Edge uv = this->getEdge(u, v);

        if (mGraph[u].getCostToCome() + mGraph[uv].getValue() < tempRHS
                //&& mGraph[u].getParent()!= v // this condition does not appear in the paper -- intended to prevent loops. but is it necessary?
            ) {
            tempRHS = mGraph[u].getCostToCome() + mGraph[uv].getValue();
            newParent = u;
        }
    }

    // Set vertex properties to discovered optimal values
    mGraph[v].setRHS(tempRHS);
    mGraph[v].setParent(newParent);
    /*
    if (std::isinf(tempRHS))
    {
        OMPL_WARN("updateVertex found no parent! %u", v);
    }
    */
    //std::cout << "After: "; printVertex(v);

    // Insert/update the OPEN queue
    // Always remove vertex from the queue
    mQueue.removeVertex(v);

    if (!isTruncated(v))
    {
        // Reinsert if it is inconsistent
        if(mGraph[v].getCostToCome()!=mGraph[v].getRHS())
            mQueue.addVertexWithKeys(v, this->calculateKeys(v));
    }
}

// ============================================================================
// Return: True if event triggered, false otherwise
bool GTLPAstar::computeShortestPath(Vertex& triggeredLeafVertex) {
  while ( mQueue.keyComparison(mQueue.getTopVertexKeys(), this->calculateKeys(mTargetVertex)) ||
          mGraph[mTargetVertex].getRHS() > mGraph[mTargetVertex].getCostToCome() ) {

    // check if the queue is empty.. which means graph has no connected path
    if (mQueue.isEmpty()){
      OMPL_INFORM("No Path Exists in the graph");
      std::cout << "Vertex Expanded: " << mNumberOfVertexExpansions
        << ", Edge Evaluated: " << mNumberOfEdgeEvaluations
        // << ", Queue Size: " <<  mQueue.getSize()
        << std::endl;
      return false;
    }

    //mQueue.printQueue();

    // Pop front vertex from the queue
    Vertex s = mQueue.popTopVertex();
    //std::cout << "Pop ";
    //printVertex(s);

    // Check truncated termination condition
    //std::cout << "T2: ";
    computeGpi(mTargetVertex);
    if (mGraph[mTargetVertex].getGpi() <=
        mEpsilon*(std::min(mGraph[s].getEstimatedTotalCost(),mGraph[s].getRHS())))
    {
      //std::cout << "T2! TLPA* terminated before expanding vertex " << s << std::endl;
      triggeredLeafVertex = mTargetVertex;
      return true;
    }
    mQueue.removeVertex(s);

    if (mGraph[s].getCostToCome() > mGraph[s].getRHS()) {
      // Make it consistent
      mGraph[s].setCostToCome(mGraph[s].getRHS());
      //std::cout << "Overconsistent! Make " << s << " consistent, update neighbors." << std::endl;

      // Count the number of expansion
      mNumberOfVertexExpansions++;

      // Now update the sucessor vertices
      NeighborIter ni, ni_end;
      for (boost::tie(ni, ni_end) = adjacent_vertices(s, mGraph); ni != ni_end; ++ni)
        this->updateVertex(*ni);

      // Triggered after vertex expansion
      // TODO: should come before/after updating neighbors?
      if (mEvent->isTriggered(s))
      {
          triggeredLeafVertex = s;
          return true;
      }
    }
    else {
      //std::cout << "T1: ";
      computeGpi(s);
      if(mGraph[s].getGpi() + mGraph[s].getHeuristic() <=
          mEpsilon*mGraph[s].getEstimatedTotalCost()) {
        //std::cout << "T1! Did not expand vertex " << s << std::endl;
        mTruncated.insert(s);

        // Per discussion with Jaien, this can trigger even though we don't expand.
        if (mEvent->isTriggered(s))
        {
            triggeredLeafVertex = s;
            return true;
        }
      }
      else {
        //std::cout << "Underconsistent! Make " << s << " overconsistent, update neighbors." << std::endl;
        // Otherwise it is underconsistent, no consistent vertices are in the queue
        // Make it overconsistent or consistent
        mGraph[s].setCostToCome(std::numeric_limits<double>::infinity());

        // Update this vertex
        this->updateVertex(s);

        // Count the number of expansion
        mNumberOfVertexExpansions++;

        // Now update the sucessor vertices
        NeighborIter ni, ni_end;
        for (boost::tie(ni, ni_end) = adjacent_vertices(s, mGraph); ni != ni_end; ++ni)
          this->updateVertex(*ni);
      }
    }
  }

  //mPlannerStatus = PlannerStatus::Solved;
  //mQueue.printQueue();
}

// ============================================================================
void GTLPAstar::computeGpi(Vertex s) {
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
Path GTLPAstar::obtainPath(Vertex s)
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
void GTLPAstar::perceiveNewWorld() {
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    // This is for identifying changed vertices.... only for approximate change detection
    this->evaluateVertex(*vi);
  }
}


bool GTLPAstar::perceiveChanges() {
  // Flush out the previously perceived changes
  mPerceivedChangedEdges.clear();

  // Reset counters;
  mNumberOfEdgeEvaluations=0;
  mNumberOfVertexExpansions=0;

  ///////////////// Perceive environment changes ////////////////////
  //// Iterate all vertices
  VertexIter vi, vi_end;
  for (boost::tie(vi, vi_end) = vertices(mGraph); vi != vi_end; ++vi) {
    // Check if vertex occupancy has changed
    ColorStatus oldStartColor = mGraph[*vi].getColorStatus();
    if (this->evaluateVertex(*vi) != oldStartColor)
    {
      // If changed, look at edges to/from all neighbors
      NeighborIter ni, ni_end;
      for (boost::tie(ni, ni_end) = adjacent_vertices(*vi, mGraph); ni != ni_end; ++ni) {
        Edge edge = this->getEdge(*ni, *vi);

        // No need to insert already unknwon edge
        if (mGraph[edge].getColorStatus() !=ColorStatus::Unknown)
        {
          mGraph[edge].setColorStatus(ColorStatus::Unknown);
          mGraph[edge].setEvaluationStatus(EvaluationStatus::NotEvaluated);
          mGraph[edge].setValue(mGraph[edge].getLength());
          mPerceivedChangedEdges.push_back(edge);
        }

        /*
        // Now if the vertex is infeasible, all incident edges are immediately infeasible.
        // But if  it's feasible, then all incident edges are now unknown
        ColorStatus previousEdgeColor = mGraph[edge].getColorStatus();
        ColorStatus newEdgeColor =
          (mGraph[*vi].getColorStatus() == ColorStatus::Infeasible)
          ? ColorStatus::Infeasible
          : ColorStatus::Unknown;

        // If it didn't change, do nothing.
        if (previousEdgeColor == newEdgeColor) continue;

        // Otherwise, update graph structure
        mPerceivedChangedEdges.push_back(edge);

        // Infeasible edges have infinite cost.
        // Unknown edges use their length as their cost.
        double edgeCost = (newEdgeColor == ColorStatus::Infeasible)
          ? std::numeric_limits<double>::infinity()
          : mGraph[edge].getLength();
        mGraph[edge].setValue(edgeCost);
        mGraph[edge].setColorStatus(newEdgeColor);
        if (newEdgeColor == ColorStatus::Unknown)
          mGraph[edge].setEvaluationStatus(EvaluationStatus::NotEvaluated);
        */
      }
    }
  }
  std::cout << "Perceived " << mPerceivedChangedEdges.size() << "changed edges" << std::endl;
  if (mPerceivedChangedEdges.empty()) return false;

  ///////////////// Prepare algorithm for changes ////////////////////
  // Update queue to handle changes
  mPlannerStatus = PlannerStatus::NotSolved;
  for (Edge e : mPerceivedChangedEdges) {
    this->updateVertex(source(e, mGraph));
    this->updateVertex(target(e, mGraph));
  }

  // Reset truncated edges for further processing
  this->clearTruncatedVertices();

  return true;
}

// ============================================================================
void GTLPAstar::setEvent(EventPtr event) {
  mEvent = event;
}

// ============================================================================
ConstEventPtr GTLPAstar::getEvent() const {
  return mEvent;
}

// ============================================================================
Edge GTLPAstar::getEdge(Vertex u, Vertex v) {
  Edge uv;
  bool edgeExists;
  boost::tie(uv, edgeExists) = edge(u, v, mGraph);
  assert(edgeExists);

  return uv;
}

// ============================================================================
Path GTLPAstar::getPathToSource(Vertex u) {
  Path pathToSource;
  while (u != mSourceVertex) {
    pathToSource.emplace_back(u);
    u = mGraph[u].getParent();
  }
  pathToSource.emplace_back(mSourceVertex);
  return pathToSource;
}

// ============================================================================
bool GTLPAstar::foundPathToGoal() {
  if (mGraph[mTargetVertex].getVisitStatus() == VisitStatus::NotVisited)
    return false;

  Path pathToGoal = getPathToSource(mTargetVertex);
  for (std::size_t i = 0; i < pathToGoal.size() - 1; ++i) {
    Edge e = getEdge(pathToGoal[i + 1], pathToGoal[i]);
    if (mGraph[e].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
      return false;
  }

  return true;
}

// ============================================================================
// TODO (avk): I should be able to set the heuristic function from the demo
// script. Create a Heuristic Class and send it in. Have a default heuristic
// if nothing has been set.
double GTLPAstar::getGraphHeuristic(Vertex v) {
  double heuristic = mSpace->distance(
      mGraph[mTargetVertex].getState()->getOMPLState(), mGraph[v].getState()->getOMPLState());
  return heuristic;
}

// ============================================================================
void GTLPAstar::setConnectionRadius(double radius) {
  mConnectionRadius = radius;
}

// ============================================================================
double GTLPAstar::getConnectionRadius() {
  return mConnectionRadius;
}

// ============================================================================
void GTLPAstar::setEpsilon(double eps) {
    mEpsilon = eps;
    assert(mEpsilon >= 1.0);
}

// ============================================================================
double GTLPAstar::getEpsilon() {
    return mEpsilon;
}

// ============================================================================
void GTLPAstar::setCollisionCheckResolution(double resolution) {
  mCollisionCheckResolution = resolution;
}

// ============================================================================
double GTLPAstar::getCollisionCheckResolution() {
  return mCollisionCheckResolution;
}

// ============================================================================
void GTLPAstar::setRoadmap(std::string filename) {
  if (filename == "")
    std::invalid_argument("Roadmap Filename cannot be empty!");

  // Load the graph.
  mRoadmap = boost::shared_ptr<io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>>(
      new io::RoadmapFromFile<Graph, VPStateMap, State, EPLengthMap>(mSpace, filename));

  mRoadmap->generate(
      mGraph, get(&VertexProperties::mState, mGraph), get(&EdgeProperties::mLength, mGraph));

  // Loop over edges, set length according to state check for consistency.
  EdgeIter ei, ei_end;
  for (boost::tie(ei, ei_end) = edges(mGraph); ei != ei_end; ++ei) {
      Vertex startVertex = source(*ei, mGraph);
      Vertex endVertex = target(*ei, mGraph);
      auto startState = mGraph[startVertex].getState()->getOMPLState();
      auto endState = mGraph[endVertex].getState()->getOMPLState();
      mGraph[*ei].setLength(mSpace->distance(startState, endState));
  }

  // Mark the graph to have been setup.
  mGraphSetup = true;
}

// ============================================================================
void GTLPAstar::setBestPathCost(double cost) {
  mBestPathCost = cost;
}

// ============================================================================
double GTLPAstar::getBestPathCost() {
  return mBestPathCost;
}

// ============================================================================
void GTLPAstar::setPlannerStatus(PlannerStatus status) {
  mPlannerStatus = status;
}

// ============================================================================
PlannerStatus GTLPAstar::getPlannerStatus() {
  return mPlannerStatus;
}

// ============================================================================
double GTLPAstar::getNumberOfEdgeEvaluations() {
  return mNumberOfEdgeEvaluations;
}

// ============================================================================
double GTLPAstar::getNumberOfEdgeRewires() {
  return mNumberOfEdgeRewires;
}

// ============================================================================
double GTLPAstar::getNumberOfVertexExpansions() {
  return mNumberOfVertexExpansions;
}

// ============================================================================
ColorStatus GTLPAstar::evaluateVertex(const Vertex& v) {
  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();

  auto state = mGraph[v].getState()->getOMPLState();

  // Evaluate the state.
  if (!validityChecker->isValid(state))
  {
    mGraph[v].setColorStatus(ColorStatus::Infeasible);
    return ColorStatus::Infeasible;
  }
  else
  {
    mGraph[v].setColorStatus(ColorStatus::Feasible);
    return ColorStatus::Feasible;
  }
}

// ============================================================================
ColorStatus GTLPAstar::evaluateEdge(const Edge& e) {
  mNumberOfEdgeEvaluations++;

  // Be optimisitic before evaluation... This will be free, this will be free...
  ColorStatus edgeColor = ColorStatus::Feasible;
  // Access the validity checker.
  auto validityChecker = si_->getStateValidityChecker();

  // Collision check the start and goal.
  Vertex startVertex = source(e, mGraph);
  Vertex endVertex = target(e, mGraph);

  auto startState = mGraph[startVertex].getState()->getOMPLState();
  auto endState = mGraph[endVertex].getState()->getOMPLState();

  // Evaluate Start and End States.
  if (!validityChecker->isValid(startState)) {
    mGraph[startVertex].setColorStatus(ColorStatus::Infeasible);
    edgeColor = ColorStatus::Infeasible;
  }
  else if (!validityChecker->isValid(endState)) {
    mGraph[endVertex].setColorStatus(ColorStatus::Infeasible);
    edgeColor = ColorStatus::Infeasible;
  }

  // Did it pass the end Vertices test?
  if (edgeColor == ColorStatus::Feasible) {
    // Evaluate the state in between.
    int maxSteps = 1.0 / mCollisionCheckResolution;
    for (int multiplier = 1; multiplier < maxSteps + 1; ++multiplier) {
      double interpolationStep = mCollisionCheckResolution * multiplier;
      assert(interpolationStep <= 1);
      StatePtr midVertex(new lgls::datastructures::State(mSpace));
      mSpace->interpolate(startState, endState, interpolationStep, midVertex->getOMPLState());

      if (!validityChecker->isValid(midVertex->getOMPLState())){
          edgeColor = ColorStatus::Infeasible;
          break;
      }
    } // End For interpolation
  } // End If edge still free

  // Now assign the edge value
  double edgeValue;
  if (edgeColor == ColorStatus::Infeasible) {
    edgeValue = std::numeric_limits<double>::infinity();
  }
  else {
    edgeValue = mSpace->distance(startState, endState);
  }

  // Actual assignment
  mGraph[e].setValue(edgeValue);
  // mGraph[e].setLength(edgeValue);
  mGraph[e].setColorStatus(edgeColor);
  mGraph[e].setEvaluationStatus(EvaluationStatus::Evaluated);

  return edgeColor;
}

// ============================================================================
ompl::base::PathPtr GTLPAstar::constructSolution(const Vertex& source, const Vertex& target) {
  ompl::geometric::PathGeometric* path = new ompl::geometric::PathGeometric(si_);
  Path p = this->obtainPath(target);
  for (Vertex v : p)
    path->append(mGraph[v].getState()->getOMPLState());
  path->reverse();
  return ompl::base::PathPtr(path);
}

// ============================================================================
void GTLPAstar::printEdge(const Edge& edge){
  ColorStatus color = mGraph[edge].getColorStatus();
  std::cout << "Edge " << edge << " is ";

  if (color == ColorStatus::Feasible) {
    std::cout << "Feasible   ";
  } else if (color == ColorStatus::Infeasible) {
    std::cout << "Infeasible ";
  } else {
    std::cout << "Unknown    ";
  }

  std::cout << ", Length: " << mGraph[edge].getLength()
            << ", Value: " << mGraph[edge].getValue()
            << std::endl;
}

void GTLPAstar::printVertex(Vertex v) {
    std::cout
        << "Vertex " << v << ": "
        << "v = " << mGraph[v].getCostToCome() << "; "
        << "g = " << mGraph[v].getRHS() << "; "
        << "h = " << mGraph[v].getHeuristic() << "; "
        << "gpi = " << mGraph[v].getGpi() << "; ";

    std::cout << "bp = ";
    if (mGraph[v].hasParent())
        std::cout << mGraph[v].getParent() << "; ";
    else
        std::cout << "null; ";

    std::cout << std::endl;
}

} // namespace lgls

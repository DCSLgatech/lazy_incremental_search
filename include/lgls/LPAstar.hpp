/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#ifndef LGLS_LPAstar_HPP_
#define LGLS_LPAstar_HPP_

// STL headers
#include <exception>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>
#include <chrono> // For high resolution clock

// Timed Callback
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>

// OMPL headers
#include <ompl/base/Planner.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/PathGeometric.h>

// LPAstar headers. Include all the headers.
#include "lgls/datastructures.hpp"
#include "lgls/io.hpp"

namespace lgls {

enum PlannerStatus { Solved, NotSolved };

/// The OMPL Planner class that implements the algorithm.
class LPAstar : public ompl::base::Planner {
public:
  /// Constructor.
  /// \param[in] si The OMPL space information manager.
  explicit LPAstar(const ompl::base::SpaceInformationPtr& si);

  /// Destructor.
  ~LPAstar(void);

  /// Callback for visualization
  void setDebugCallback(std::function<void(lgls::datastructures::Graph g)> callback);
  std::function<void(lgls::datastructures::Graph g)> mCallback;

  /// Setup the planner.
  void setup() override;

  /// Set the problem definition and define the start, goal.
  /// \param[in] pdef OMPL Problem Definition.
  void setProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef) override;

  /// Solve the planning problem.
  /// \param[in] ptc OMPL Planning Termination Condition.
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& ptc);

  /// Clear the planner setup.
  void clear() override;

  /// Set the connection radius of the graph.
  void setConnectionRadius(double radius);

  /// Get the connection radius of the graph.
  double getConnectionRadius();

  /// Set the collision checking resolution along the edge.
  void setCollisionCheckResolution(double resolution);

  /// Get the connection radius of the graph.
  double getCollisionCheckResolution();

  /// Set the roadmap. Loads the graph.
  void setRoadmap(std::string filename);

  /// Set the best path cost.
  void setBestPathCost(double cost);

  /// Get the best path cost.
  double getBestPathCost();

  /// Set status of the planner.
  void setPlannerStatus(PlannerStatus);

  /// Get status of the planner.
  PlannerStatus getPlannerStatus();

  /// Get the number of edges evaluated.
  double getNumberOfEdgeEvaluations();

  /// Get the number of vertex expansion.
  double getNumberOfVertexExpansions();

  /// Evaluate all vertices to identify the differences next time
  void perceiveNewWorld();

  /// Identify all changed vertices and update their edges, then updateVertex
  bool perceiveChanges();

  /// For visuaization of graph
  lgls::datastructures::Graph getGraph(){return mGraph;};

  std::vector<lgls::datastructures::Edge> getPerceivedChangedEdges(){return mPerceivedChangedEdges;};

private:
  /// Adds source and target vertices, and relevant edges to \c mGraph.
  /// Sets up the event and the selector.
  void setupPreliminaries();

  /// Returns edge between source and target vertices.
  lgls::datastructures::Edge getEdge(lgls::datastructures::Vertex, lgls::datastructures::Vertex);

  /// Heuristic function.
  double getGraphHeuristic(lgls::datastructures::Vertex v);

  /// Evaluates an edge for collision.
  lgls::datastructures::CollisionStatus evaluateVertex(const lgls::datastructures::Vertex& v);

  /// Evaluates an edge for collision.
  lgls::datastructures::CollisionStatus evaluateEdge(const lgls::datastructures::Edge& e);

  /// Calculate Keys for vertex
  lgls::datastructures::Keys calculateKeys(lgls::datastructures::Vertex v);

  /// Update Vertex find a best parent
  void updateVertex(lgls::datastructures::Vertex v);

  /// Compute shortest path
  void computeShortestPath();

  /// Return the path from source to target vertices.
  ompl::base::PathPtr constructSolution(
      const lgls::datastructures::Vertex&, const lgls::datastructures::Vertex&);

  /// The pointer to the OMPL state space.
  const ompl::base::StateSpacePtr mSpace;

  /// Boost roadmap representation.
  boost::shared_ptr<io::RoadmapFromFile<
      lgls::datastructures::Graph,
      lgls::datastructures::VPStateMap,
      lgls::datastructures::State,
      lgls::datastructures::EPLengthMap>>
      mRoadmap;

  /// Connection radius in the graph.
  double mConnectionRadius;

  /// Collision checking resolution for the edge.
  double mCollisionCheckResolution;

  /// Boolean denoting if the graph has been setup.
  bool mGraphSetup{false};

  /// Filename containing the roadmap.
  double mBestPathCost{std::numeric_limits<double>::infinity()};

  /// Flag to check if the planner succeeded.
  PlannerStatus mPlannerStatus{PlannerStatus::NotSolved};

  /// Queue representing the inconsistent vertices to expand.
  lgls::datastructures::Queue mQueue;

  /// The fixed roadmap over which the search is done.
  lgls::datastructures::Graph mGraph;

  /// Source vertex.
  lgls::datastructures::Vertex mSourceVertex;

  /// Target vertex.
  lgls::datastructures::Vertex mTargetVertex;

  /// TODO (avk): Move these into PlannerStatus class.
  /// Number of Edge Evaluations.
  double mNumberOfEdgeEvaluations{0};

  /// Number of Vertex Expansions
  double mNumberOfVertexExpansions{0};

  std::vector<lgls::datastructures::Edge> mPerceivedChangedEdges;

  /// For timing and visualization
  double mTotalEdgeEvaluationTime{0};
  double mTotalVertexExpansionTime{0};

  bool mDebugging{false};
  bool mJoined{false};
  bool mDraw{false};
  void call_visualize();

  std::mutex mtx;
  std::condition_variable cv{};
  std::chrono::microseconds time{100};

};

} // namespace lgls

#endif // LPAstar_LPAstar_HPP_

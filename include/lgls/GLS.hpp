/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef LGLS_GLS_HPP_
#define LGLS_GLS_HPP_

// STL headers
#include <exception>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>
#include <chrono> // For high resolution clock

// OMPL headers
#include <ompl/base/Planner.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/geometric/PathGeometric.h>

// GLS headers. Include all the headers.
#include "lgls/datastructures.hpp"
#include "lgls/event.hpp"
#include "lgls/io.hpp"
#include "lgls/selector.hpp"

namespace lgls {

enum TreeValidityStatus { Valid, NotValid };

// enum PlannerStatus { Solved, NotSolved };

/// The OMPL Planner class that implements the algorithm.
class GLS : public ompl::base::Planner {
public:
  /// Constructor.
  /// \param[in] si The OMPL space information manager.
  explicit GLS(const ompl::base::SpaceInformationPtr& si);

  /// Destructor.
  ~GLS(void);

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

  /// Set the event to be used by GLS.
  /// \param[in] event Event that defines the trigger condition.
  void setEvent(lgls::event::EventPtr event);

  /// Returns the event used by the algorithm.
  lgls::event::ConstEventPtr getEvent() const;

  /// Set the selector to be used by GLS.
  /// \param[in] selector Selector that defines the evaluation strategy.
  void setSelector(lgls::selector::SelectorPtr selector);

  /// Returns the selector used by the algorithm.
  lgls::selector::ConstSelectorPtr getSelector() const;

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

  /// Get the number of vertex expansions.
  double getNumberOfVertexExpansions();

  /// Get the number of edges rewired.
  double getNumberOfEdgeRewires();

private:
  /// Adds source and target vertices, and relevant edges to \c mGraph.
  /// Sets up the event and the selector.
  void setupPreliminaries();

  /// Returns edge between source and target vertices.
  lgls::datastructures::Edge getEdge(lgls::datastructures::Vertex, lgls::datastructures::Vertex);

  /// Returns the path from vertex to source.
  lgls::datastructures::Path getPathToSource(lgls::datastructures::Vertex);

  /// Returns true if the path to goal is collision-free.
  bool foundPathToGoal();

  /// Heuristic function.
  double getGraphHeuristic(lgls::datastructures::Vertex v);

  /// Evaluates an edge for collision.
  lgls::datastructures::CollisionStatus evaluateVertex(lgls::datastructures::Vertex v);

  /// Evaluates an edge for collision.
  lgls::datastructures::CollisionStatus evaluateEdge(const lgls::datastructures::Edge& e);

  /// Extends the search tree forwards.
  void extendSearchTree();

  /// Updates the vertex properties in the search tree.
  void updateSearchTree();

  /// Rewires the search tree when edge costs change.
  void rewireSearchTree();

  /// Evaluates the search tree when the extension pauses.
  void evaluateSearchTree();

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
  double mBestPathCost{0};

  /// Flag to check if the planner succeeded.
  PlannerStatus mPlannerStatus{PlannerStatus::NotSolved};

  /// Flag to check the validity of the search tree.
  TreeValidityStatus mTreeValidityStatus{TreeValidityStatus::Valid};

  /// SearchQueue representing the open list to extend.
  lgls::datastructures::SearchQueue mExtendQueue;

  /// SearchQueue representing the vertices whose attirbutes need update.
  lgls::datastructures::SearchQueue mUpdateQueue;

  /// SearchQueue representing the search tree that needs repairing.
  lgls::datastructures::SearchQueue mRewireQueue;

  /// Set of vertices used for rewiring.
  std::set<lgls::datastructures::Vertex> mRewireSet;

  /// Event
  lgls::event::EventPtr mEvent;

  /// Selector
  lgls::selector::SelectorPtr mSelector;

  /// The fixed roadmap over which the search is done.
  lgls::datastructures::Graph mGraph;

  /// Source vertex.
  lgls::datastructures::Vertex mSourceVertex;

  /// Target vertex.
  lgls::datastructures::Vertex mTargetVertex;

  /// TODO (avk): Move these into PlannerStatus class.
  /// Number of Edge Evaluations.
  double mNumberOfEdgeEvaluations{0};

  /// Number of Edge Evaluations.
  double mNumberOfVertexExpansions{0};

  /// Number of Edge Rewires.
  double mNumberOfEdgeRewires{0};

  /// For timing
  double mTotalEdgeEvaluationTime{0};
  double mTotalVertexExpansionTime{0};
};

} // namespace lgls

#endif // LGLS_GLS_HPP_

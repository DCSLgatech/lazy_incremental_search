/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#ifndef LGLS_DATASTRUCTURES_GRAPH_HPP_
#define LGLS_DATASTRUCTURES_GRAPH_HPP_

// STL headers
#include <set>
#include <vector>

// Boost headers
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/dynamic_property_map.hpp>

// LGLS headers
#include "lgls/datastructures/State.hpp"
#include "lgls/datastructures/Types.hpp"

// TODO (avk): state and length are made public to accomodate the
// roadmapmanager which seems stupid. Change it if possible.

namespace lgls {
namespace datastructures {


enum CollisionStatus { Collision, Free};

enum EvaluationStatus { NotEvaluated, Evaluated };

class VertexProperties {
public:
  // Set state wrapper around underlying OMPL state.
  void setState(StatePtr state);

  // Get state wrapper around underlying OMPL state.
  StatePtr getState();

  // Set cost-to-come.
  void setCostToCome(double cost);

  // Get cost-to-come.
  double getCostToCome();

  // Set cost-to-come rhs-value.
  void setRHS(double cost);

  // Get cost-to-come rhs-value.
  double getRHS();

  // Set heuristic.
  void setHeuristic(double heuristic);

  // Get heuristic.
  double getHeuristic();

  // Set the vertex parent.
  void setParent(Vertex parent);

  // Get the vertex parent.
  Vertex getParent();

  // Get true if parent exists
  bool hasParent();

  // Removes parent, change hasParent to False
  void removeParent();

  // Set the cost of path Pi.
  void setGpi(double gpi);

  // Get the cost of path Pi.
  double getGpi();

  // Set the path Pi.
  void setPi(const std::vector<Vertex>& pi);

  // Get the vertex parent.
  std::vector<Vertex> getPi();

  // Get true if parent exists
  bool hasPi();

  // Removes parent, change hasParent to False
  void removePi();

  // Sets the evaluation status.
  void setEvaluationStatus(EvaluationStatus evaluationStatus);

  // Get the evaluation status.
  EvaluationStatus getEvaluationStatus();

  // Sets the collision status of the vertex.
  void setCollisionStatus(CollisionStatus status);

  // Get the collision status of the vertex.
  CollisionStatus getCollisionStatus();

  /// Underlying state.
  /// TODO (avk): why is this public?
  StatePtr mState;

private:
  /// Cost-to-Come g-value.
  double mCostToCome{std::numeric_limits<double>::infinity()};

  /// Cost-to-Come rhs-value.
  double mRHS{std::numeric_limits<double>::infinity()};

  /// Heuristic value.
  double mHeuristic{std::numeric_limits<double>::infinity()};

  /// Parent.
  Vertex mParent{boost::graph_traits<BasicGraph>::null_vertex()};

  /// Gpi value for TLPA* (path cost of Pi)
  double mGpi{std::numeric_limits<double>::infinity()};

  /// Path from vertex to start. Used to compute truncation rules
  std::vector<Vertex> mPi;

  /// Does it have a Pi?
  bool mHasPi{false};

  /// Evaluation status.
  EvaluationStatus mEvaluationStatus{EvaluationStatus::NotEvaluated};

  /// Collision status.
  CollisionStatus mCollisionStatus{CollisionStatus::Free};

};

class EdgeProperties {
public:
  // Sets the length of the edge. As Heuristic Distance
  void setLength(double length);

  // Get the length of the edge.
  double getLength();

  /// Sets the value of the edge.
  void setValue(double value);

  /// Get the value of the edge.
  double getValue();

  /// Get the value of the edge with inflated heuristic.
  double getValue(double inflation);

  // Sets the evaluation status.
  void setEvaluationStatus(EvaluationStatus evaluationStatus);

  // Get the evaluation status.
  EvaluationStatus getEvaluationStatus();

  // Sets the collision status.
  void setCollisionStatus(CollisionStatus status);

  // Get the collision status.
  CollisionStatus getCollisionStatus();

  /// The length of the edge using the space distance metric.
  /// TODO (avk): Why is this public?
  double mLength;



private:
  /// Edge Value.
  double mValue;

  bool mValueEvaluated{false};

  /// Evaluation status.
  EvaluationStatus mEvaluationStatus{EvaluationStatus::NotEvaluated};

  /// Collision status..
  CollisionStatus mCollisionStatus{CollisionStatus::Free};

};

/// Undirected Boost graph using the properties just defined.
typedef boost::
    adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties>
        Graph;

/// Shared pointer to Graph.
typedef std::shared_ptr<Graph> GraphPtr;

/// Shared pointer to const Graph.
typedef std::shared_ptr<const Graph> ConstGraphPtr;

/// Boost vertex iterator
typedef boost::graph_traits<Graph>::vertex_iterator VertexIter;

/// Boost edge iterator
typedef boost::graph_traits<Graph>::edge_iterator EdgeIter;

/// Boost graph neighbor iterator
typedef boost::graph_traits<Graph>::adjacency_iterator NeighborIter;

/// Map each vertex to the underlying state [read from the graphml file]
typedef boost::property_map<Graph, lgls::datastructures::StatePtr VertexProperties::*>::type
    VPStateMap;

/// Map each edge to its length
typedef boost::property_map<Graph, double EdgeProperties::*>::type EPLengthMap;

} // namespace datastructures
} // namespace lgls

#endif // LGLS_DATASTRUCTURES_GRAPH_HPP_

#ifndef LGLS_EVENT_SUBPATHEXISTENCEEVENT_HPP_
#define LGLS_EVENT_SUBPATHEXISTENCEEVENT_HPP_

#include <unordered_map>

#include "lgls/event/Event.hpp"

namespace lgls {
namespace event {

/// Unordered map defined as: <source name> <target name> <prior>
typedef std::unordered_map<
    std::pair<std::size_t, std::size_t>,
    double,
    boost::hash<std::pair<std::size_t, std::size_t>>>
    edgeToPriorMap;

/// Event that triggers when the search tree reaches below given
/// a threshold in probability of existence.
/// Additionally, the event also triggers when the vertex is the target.
class SubPathExistenceEvent : public Event {
public:
  /// Constructor.
  SubPathExistenceEvent(edgeToPriorMap& priorMap, double existenceThreshold);

  /// Documentation inherited.
  bool isTriggered(const lgls::datastructures::Vertex& vertex) override;

  /// Documentation inherited.
  void updateVertexProperties(lgls::datastructures::Vertex& vertex) override;

private:
  /// Get the probability of the path to the vertex.
  double getExistenceProbability(const lgls::datastructures::Vertex& vertex);

  /// Add vertex to the map.
  void addVertexToMap(lgls::datastructures::Vertex& vertex);

  /// Evaluate the prior of given edge.
  double getPrior(const lgls::datastructures::Edge& edge);

  /// Map that stores the priors.
  edgeToPriorMap mPriorMap;

  /// The threshold over depth.
  std::size_t mExistenceThreshold;

  /// The map from vertex to depth in the search tree.
  std::unordered_map<lgls::datastructures::Vertex, double> mSubPathExistenceMap;
};

} // namespace event
} // namespace lgls

#endif // LGLS_EVENT_SUBPATHEXISTENCEEVENT_HPP_

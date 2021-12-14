#ifndef LGLS_EVENT_CONSTANTDEPTHEVENT_HPP_
#define LGLS_EVENT_CONSTANTDEPTHEVENT_HPP_

#include <unordered_map>

#include "lgls/event/Event.hpp"

namespace lgls {
namespace event {

/// Event that triggers when the search tree reaches a particular depth.
/// Additionally, the event also triggers when the vertex is the target.
class ConstantDepthEvent : public Event {
public:
  /// Constructor.
  explicit ConstantDepthEvent(std::size_t depth);

  /// Documentation inherited.
  bool isTriggered(const lgls::datastructures::Vertex& vertex) override;

  bool isTriggered(const lgls::datastructures::Path& path) override;

  /// Documentation inherited.
  void updateVertexProperties(lgls::datastructures::Vertex& vertex) override;

  void setDepth(std::size_t depth){mDepthThreshold = depth;};

private:
  /// Get the depth of the vertex.
  std::size_t getDepth(const lgls::datastructures::Vertex& vertex);

  /// The threshold over depth.
  std::size_t mDepthThreshold;

};

} // namespace event
} // namespace lgls

#endif // LGLS_EVENT_CONSTANTDEPTHEVENT_HPP_

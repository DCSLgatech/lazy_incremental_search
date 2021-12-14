#ifndef LGLS_EVENT_SHORTESTPATHEVENT_HPP_
#define LGLS_EVENT_SHORTESTPATHEVENT_HPP_

#include "lgls/event/Event.hpp"

namespace lgls {
namespace event {

/// Event that triggers when a shortest path to goal is found.
class ShortestPathEvent : public Event {
public:
  /// Constructor.
  ShortestPathEvent();

  /// Documentation inherited.
  bool isTriggered(const lgls::datastructures::Vertex& vertex) override;

  bool isTriggered(const lgls::datastructures::Path& path) override;

  /// Documentation inherited.
  void updateVertexProperties(lgls::datastructures::Vertex& vertex) override;
};

} // namespace event
} // namespace lgls

#endif // LGLS_EVENT_SHORTESTPATHEVENT_HPP_

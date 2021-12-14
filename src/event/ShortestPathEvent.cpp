#include "lgls/event/ShortestPathEvent.hpp"

namespace lgls {
namespace event {

using lgls::datastructures::Graph;
using lgls::datastructures::Vertex;
using lgls::datastructures::Path;

//==============================================================================
ShortestPathEvent::ShortestPathEvent() {
  // Do nothing.
}

//==============================================================================
bool ShortestPathEvent::isTriggered(const Vertex& vertex) {
  if (vertex == mTargetVertex) {
    return true;
  }
  return false;
}

//==============================================================================
bool ShortestPathEvent::isTriggered(const Path& path) {
  if (path[0] == mTargetVertex) {
    return true;
  }
  return false;
}

//==============================================================================
void ShortestPathEvent::updateVertexProperties(Vertex& /*vertex*/) {
  // Do nothing.
}

} // namespace event
} // namespace lgls

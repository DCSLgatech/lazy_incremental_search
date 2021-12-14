/* Authors: Aditya Vamsikrishna Mandalika */

#include "lgls/event/Event.hpp"

namespace lgls {
namespace event {

using lgls::datastructures::Graph;
using lgls::datastructures::Vertex;

//==============================================================================
Event::Event() {
  // Do nothing.
}

//==============================================================================
void Event::setup(Graph* graph, Vertex& source, Vertex& target) {
  mGraph = graph;
  mSourceVertex = source;
  mTargetVertex = target;
}


} // namespace event
} // namespace lgls

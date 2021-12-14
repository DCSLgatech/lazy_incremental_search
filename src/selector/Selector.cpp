/* Authors: Aditya Vamsikrishna Mandalika */

#include "lgls/selector/Selector.hpp"

namespace lgls {
namespace selector {

using lgls::datastructures::Graph;

//==============================================================================
Selector::Selector() {
  // Do nothing.
}

//==============================================================================
void Selector::setup(Graph* graph) {
  mGraph = graph;
}

} // namespace selector
} // namespace lgls

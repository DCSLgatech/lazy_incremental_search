#include "lgls/selector/ForwardSelector.hpp"

namespace lgls {
namespace selector {

using lgls::datastructures::Edge;
using lgls::datastructures::EvaluationStatus;
using lgls::datastructures::Path;

//==============================================================================
ForwardSelector::ForwardSelector() {
  // Do nothing.
}

//==============================================================================
Edge ForwardSelector::selectEdgeToEvaluate(Path path) {
  // Access the graph.
  auto graph = *mGraph;
  Edge edgeToEvaluate;

  // Return the first unevaluated edge closest to source.
  for (std::size_t i = path.size() - 1; i > 0; --i) {
    bool edgeExists;
    boost::tie(edgeToEvaluate, edgeExists) = edge(path[i], path[i - 1], graph);

    if (graph[edgeToEvaluate].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
      break;
  }

  return edgeToEvaluate;
}

} // namespace selector
} // namespace lgls

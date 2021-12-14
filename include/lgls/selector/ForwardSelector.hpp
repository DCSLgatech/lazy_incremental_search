#ifndef LGLS_SELECTOR_FORWARDSELECTOR_HPP_
#define LGLS_SELECTOR_FORWARDSELECTOR_HPP_

#include "lgls/selector/Selector.hpp"

namespace lgls {
namespace selector {

/// Selector that evaluates the edge on the path closest to the source.
class ForwardSelector : public Selector {
public:
  /// Constructor.
  ForwardSelector();

  /// Documentation inherited.
  lgls::datastructures::Edge selectEdgeToEvaluate(lgls::datastructures::Path path) override;
};

} // namespace selector
} // namespace lgls

#endif // LGLS_SELECTOR_FORWARDSELECTOR_HPP_

#ifndef LGLS_SELECTOR_BACKWARDSELECTOR_HPP_
#define LGLS_SELECTOR_BACKWARDSELECTOR_HPP_

#include "lgls/selector/Selector.hpp"

namespace lgls {
namespace selector {

/// Selector that evaluates the edge on the path closest to the target.
class BackwardSelector : public Selector {
public:
  /// Constructor.
  BackwardSelector();

  /// Documentation inherited.
  lgls::datastructures::Edge selectEdgeToEvaluate(lgls::datastructures::Path path) override;
};

} // namespace selector
} // namespace lgls

#endif // LGLS_SELECTOR_BACKWARDSELECTOR_HPP_

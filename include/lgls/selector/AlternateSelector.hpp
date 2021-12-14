#ifndef LGLS_SELECTOR_ALTERNATESELECTOR_HPP_
#define LGLS_SELECTOR_ALTERNATESELECTOR_HPP_

#include "lgls/selector/Selector.hpp"

namespace lgls {
namespace selector {

/// Selector alternates between forward and backward selector.
/// By default the selector begins with forward and then flips.
class AlternateSelector : public Selector {
public:
  /// Constructor.
  AlternateSelector();

  /// Documentation inherited.
  lgls::datastructures::Edge selectEdgeToEvaluate(lgls::datastructures::Path path) override;

private:
  /// Iteration index to switch between forward and backward.
  bool mUseForwardSelector{false};

  /// Forward Selector.
  lgls::selector::SelectorPtr mForwardSelector;

  /// Backward Selector.
  lgls::selector::SelectorPtr mBackwardSelector;
};

} // namespace selector
} // namespace lgls

#endif // LGLS_SELECTOR_ALTERNATESELECTOR_HPP_

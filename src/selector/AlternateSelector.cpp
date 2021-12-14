#include "lgls/selector/AlternateSelector.hpp"
#include "lgls/selector/BackwardSelector.hpp"
#include "lgls/selector/ForwardSelector.hpp"

namespace lgls {
namespace selector {

using lgls::datastructures::Edge;
using lgls::datastructures::Path;

//==============================================================================
AlternateSelector::AlternateSelector() {
  // Setup forward selector.
  mForwardSelector = std::make_shared<lgls::selector::ForwardSelector>();
  mForwardSelector->setup(mGraph);

  // Setup backward selector.
  mBackwardSelector = std::make_shared<lgls::selector::BackwardSelector>();
  mBackwardSelector->setup(mGraph);
}

//==============================================================================
Edge AlternateSelector::selectEdgeToEvaluate(Path path) {
  // Flip the boolean.
  mUseForwardSelector = !mUseForwardSelector;

  if (mUseForwardSelector)
    return mForwardSelector->selectEdgeToEvaluate(path);
  else
    return mBackwardSelector->selectEdgeToEvaluate(path);
}

} // namespace selector
} // namespace lgls

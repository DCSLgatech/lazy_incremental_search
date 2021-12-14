/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef LGLS_SELECTOR_FAILFASTSELECTOR_HPP_
#define LGLS_SELECTOR_FAILFASTSELECTOR_HPP_

#include <unordered_map>

#include "lgls/selector/Selector.hpp"

namespace lgls {
namespace selector {

/// Unordered map defined as: <source name> <target name> <prior>
typedef std::unordered_map<
    std::pair<std::size_t, std::size_t>,
    double,
    boost::hash<std::pair<std::size_t, std::size_t>>>
    edgeToPriorMap;

/// Selector that evaluates the edge most likely to be in collision.
class FailFastSelector : public Selector {
public:
  /// Constructor.
  FailFastSelector(edgeToPriorMap& priorMap);

  /// Documentation inherited.
  lgls::datastructures::Edge selectEdgeToEvaluate(lgls::datastructures::Path path) override;

protected:
  /// Evaluate the prior of given edge.
  double getPrior(lgls::datastructures::Edge edge);

  /// Map that stores the priors.
  edgeToPriorMap mPriorMap;
};

} // namespace selector
} // namespace lgls

#endif // LGLS_SELECTOR_FAILFASTSELECTOR_HPP_

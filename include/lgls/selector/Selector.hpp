/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef LGLS_SELECTOR_SELECTOR_HPP_
#define LGLS_SELECTOR_SELECTOR_HPP_

#include <string>  // std::string
#include <utility> // std::pait
#include <vector>  // std::vector

#include "lgls/datastructures/Graph.hpp"
#include "lgls/datastructures/Types.hpp"

namespace lgls {
namespace selector {

/// Selector is a base class for selecting edges to evaluate.
/// The rule for selecting the edges to evaluate is specified
/// by the concrete classes.
class Selector {
public:
  /// Constructor.
  Selector();

  /// Destructor.
  ~Selector() = default;

  /// Setup the selector with required internal data members.
  /// \param[in] graph Graph the selector is operating with.
  void setup(lgls::datastructures::Graph* graph);

  /// Selects edges to evaluate from given path.
  /// \param[in] path The list of vertices along the path.
  /// The vertices are from leaf to source.
  virtual lgls::datastructures::Edge selectEdgeToEvaluate(lgls::datastructures::Path path) = 0;

protected:
  /// Pointer to the graph.
  lgls::datastructures::Graph* mGraph;

}; // Selector

typedef std::shared_ptr<Selector> SelectorPtr;
typedef std::shared_ptr<const Selector> ConstSelectorPtr;

} // namespace selector
} // namespace lgls

#endif // LGLS_SELECTOR_SELECTOR_HPP_

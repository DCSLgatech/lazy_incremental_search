/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef LGLS_DATASTRUCTURES_STATE_HPP_
#define LGLS_DATASTRUCTURES_STATE_HPP_

#include <ompl/base/StateSpace.h>

namespace lgls {
namespace datastructures {

class State {
public:
  /// Constructor.
  State();

  /// Constructor.
  /// \param[in] space Statespace the state belongs to.
  explicit State(ompl::base::StateSpacePtr space);

  /// Destructor.
  ~State();

  /// Get OMPL state.
  // TODO (avk): Should I make this function const?
  ompl::base::State* getOMPLState();

  /// Get OMPL space.
  // TODO (avk): Should I make this function const?
  ompl::base::StateSpacePtr getStateSpace();

private:
  /// The OMPL statespace operating on.
  const ompl::base::StateSpacePtr mSpace;

  /// The OMPL state that is being wrapped.
  ompl::base::State* mState;
};

/// Shared pointer to the State.
typedef std::shared_ptr<State> StatePtr;

/// Shared pointer to const State
typedef std::shared_ptr<const State> ConstStatePtr;

} // namespace datastructures
} // namespace lgls

#endif // LGLS_DATASTRUCTURES_STATE_HPP_

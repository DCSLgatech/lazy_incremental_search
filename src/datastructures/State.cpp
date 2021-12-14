/* Authors: Aditya Vamsikrishna Mandalika */

#include "lgls/datastructures/State.hpp"

namespace lgls {
namespace datastructures {

// ================================================================================================
State::State() {
  // Do nothing.
}

// ================================================================================================
State::State(ompl::base::StateSpacePtr space) : mSpace(space), mState(space->allocState()) {
  // Do nothing.
}

// ================================================================================================
State::~State() {
  mSpace->freeState(this->mState);
}

// ================================================================================================
ompl::base::State* State::getOMPLState() {
  return mState;
}

// ================================================================================================
ompl::base::StateSpacePtr State::getStateSpace() {
  return mSpace;
}

} // namespace datastructures
} // namespace lgls

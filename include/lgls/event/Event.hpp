/* Authors: Aditya Vamsikrishna Mandalika */

#ifndef LGLS_EVENT_EVENT_HPP_
#define LGLS_EVENT_EVENT_HPP_

#include <string>  // std::string
#include <utility> // std::pair
#include <vector>  // std::vector

#include "lgls/datastructures/Graph.hpp"
#include "lgls/datastructures/Types.hpp"

namespace lgls {
namespace event {

enum vertexUpdateOption { SingleUpdate, CascadeUpdate };

/// Event is a base class to define the trigger to pause search.
/// The rule for switching between serach and edge evaluation is
/// specified by the concrete classes.
class Event {
public:
  /// Constructor.
  Event();

  /// Destructor.
  virtual ~Event() = default;

  /// Setup the event with required internal data members.
  /// \param[in] graph Graph the event is operating with.
  /// \param[in] source Source vertex in the graph the event is attached to.
  /// \param[in] target Target vertex in the graph the event is attached to.
  void setup(
      lgls::datastructures::Graph* graph,
      lgls::datastructures::Vertex& source,
      lgls::datastructures::Vertex& target);

  /// Return true if the event is triggered.
  /// \param[in] vertex Vertex that might cause the trigger.
  virtual bool isTriggered(const lgls::datastructures::Vertex& vertex) = 0;

  virtual bool isTriggered(const lgls::datastructures::Path& path) = 0;

  /// Update vertex properties
  /// Concrete classes specify the appropriate update rules.
  /// \param[in] vertex Vertex whose properties need to be updated.
  /// downstream.
  virtual void updateVertexProperties(lgls::datastructures::Vertex& vertex) = 0;


protected:
  /// Pointer to the graph.
  lgls::datastructures::Graph* mGraph;

  /// Source vertex of the graph.
  lgls::datastructures::Vertex mSourceVertex;

  /// Target vertex of the graph.
  lgls::datastructures::Vertex mTargetVertex;

}; // Event

typedef std::shared_ptr<Event> EventPtr;
typedef std::shared_ptr<const Event> ConstEventPtr;

} // namespace event
} // namespace lgls

#endif // LGLS_EVENT_EVENT_HPP_

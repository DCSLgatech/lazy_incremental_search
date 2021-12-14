/* Author: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#ifndef LGLS_DATASTRUCTURES_QUEUE_HPP_
#define LGLS_DATASTRUCTURES_QUEUE_HPP_

// STL headers
#include <functional> // std::function
#include <set>        // std::set

// OMPL headers
#include <ompl/base/Cost.h>
#include <ompl/datastructures/BinaryHeap.h>

// LGLS headers
#include "lgls/datastructures/Types.hpp"

namespace lgls {
namespace datastructures {

class Queue {
public:

  /// The function signature of the sorting function for the vertex queue.
  typedef std::function<bool(
      const std::pair<lgls::datastructures::Vertex, lgls::datastructures::Keys >&,
      const std::pair<lgls::datastructures::Vertex, lgls::datastructures::Keys >&)>
      VertexSortingFunction;

  /// The underlying vertex queue.
  typedef std::set<std::pair<lgls::datastructures::Vertex,lgls::datastructures::Keys>, VertexSortingFunction>
      VertexQueue;

  /// Constructor.
  Queue();

  /// Destructor.
  virtual ~Queue() = default;

  /// Clear the search queue.
  void clear();

  /// Adds vertex and value to search queue.
  /// \param[in] vertex Vertex to remove from the queue.
  /// \param[in] cost Cost the vertex is ties to.
  // void addVertexWithValue(lgls::datastructures::Vertex vertex, double cost);

  void addVertexWithKeys(lgls::datastructures::Vertex vertex, lgls::datastructures::Keys);

  /// Pop top vertex.
  lgls::datastructures::Vertex popTopVertex();

  /// Get top vertex. Does not remove from the queue.
  lgls::datastructures::Vertex getTopVertex();

  /// Get top vertex value.
  lgls::datastructures::Keys getTopVertexKeys();

  /// Remove vertex from search queue.
  void removeVertex(const lgls::datastructures::Vertex vertex);
  /// \param[in] vertex Vertex to remove from the queue.
  /// \param[in] cost Cost associated with the vertex.
  // void removeVertexWithValue(const lgls::datastructures::Vertex vertex, double cost);

  /// Returns true if queue is empty.
  bool isEmpty();

  /// Returns the size of the queue.
  std::size_t getSize() const;

  bool keyComparison(
      const lgls::datastructures::Keys& left,
      const lgls::datastructures::Keys& right) const;

  void printQueue() const;

private:
  /// Custom comparator used to order vertices.
  bool queueComparison(
      const std::pair<lgls::datastructures::Vertex, lgls::datastructures::Keys>&,
      const std::pair<lgls::datastructures::Vertex, lgls::datastructures::Keys>&) const;

  /// The underlying queue of vertices sorted by VertexQueueSortingFunction.
  VertexQueue mVertexQueue;

}; // Queue

} // namespace datastructures
} // namespace lgls

#endif // LGLS_DATASTRUCTURES_QUEUE_HPP_

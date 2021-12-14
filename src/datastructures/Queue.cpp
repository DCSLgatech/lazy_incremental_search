/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#include "lgls/datastructures/Queue.hpp"

#include <ompl/util/Console.h> // OMPL_INFORM

namespace lgls {
namespace datastructures {

using lgls::datastructures::Vertex;

Queue::Queue()
  : mVertexQueue([this](
                     const std::pair<lgls::datastructures::Vertex, lgls::datastructures::Keys>& lhs,
                     const std::pair<lgls::datastructures::Vertex, lgls::datastructures::Keys>& rhs) {
    return queueComparison(lhs, rhs);
  }) {
  // Do Nothing.
}

// ============================================================================
void Queue::clear() {
  mVertexQueue.clear();
}

// ============================================================================
// void Queue::addVertexWithValue(Vertex vertex, double cost) {
//   std::pair<Vertex, double> addPair = std::make_pair(vertex, cost);
//   mVertexQueue.insert(addPair);
// }

void Queue::addVertexWithKeys(Vertex vertex, Keys keys) {
  // Keys keys = std::make_pair(key1,key2);
  std::pair<Vertex, Keys> addPair = std::make_pair(vertex, keys);
  mVertexQueue.insert(addPair);
}


// ============================================================================
Vertex Queue::popTopVertex() {
  Vertex topVertex = (*mVertexQueue.begin()).first;
  mVertexQueue.erase(mVertexQueue.begin());

  return topVertex;
}

// ============================================================================
Vertex Queue::getTopVertex() {
  return (*mVertexQueue.begin()).first;
}

// ============================================================================
Keys Queue::getTopVertexKeys() {
  return (*mVertexQueue.begin()).second;
}

// ============================================================================
// void Queue::removeVertexWithValue(Vertex vertex, double cost) {
//   auto iterQ = mVertexQueue.find(std::make_pair(vertex, cost));
//   if (iterQ != mVertexQueue.end())
//     mVertexQueue.erase(iterQ);
// }

// ============================================================================
void Queue::removeVertex(Vertex vertex) {
  auto iterQ = std::find_if(mVertexQueue.begin(), mVertexQueue.end(),
          [&vertex](const std::pair<Vertex,Keys>& p ){ return p.first == vertex; });
  if (iterQ != mVertexQueue.end())
    mVertexQueue.erase(iterQ);
}


// ============================================================================
bool Queue::isEmpty() {
  if (mVertexQueue.empty())
    return true;

  return false;
}

// ============================================================================
std::size_t Queue::getSize() const {
  return mVertexQueue.size();
}

// ============================================================================
// bool Queue::hasVertexWithValue(const Vertex vertex, double cost) {
//   auto iterQ = mVertexQueue.find(std::make_pair(vertex, cost));
//   if (iterQ != mVertexQueue.end())
//     return true;
//
//   return false;
// }
// ============================================================================
bool Queue::keyComparison(
    const lgls::datastructures::Keys& left,
    const lgls::datastructures::Keys& right) const {

  if (left.first < right.first)
    return true;
  else if (left.first > right.first)
    return false;
  else
  {
    if (left.second < right.second)
      return true;
    else
      return false;
  }

}

// ============================================================================
bool Queue::queueComparison(
    const std::pair<lgls::datastructures::Vertex, lgls::datastructures::Keys>& left,
    const std::pair<lgls::datastructures::Vertex, lgls::datastructures::Keys>& right) const {

  return this->keyComparison(left.second, right.second);
  // if (left.second.first < right.second.first)
  //   return true;
  // else if (left.second.first > right.second.first)
  //   return false;
  // else
  // {
  //   if (left.second.second < right.second.second)
  //     return true;
  //   else
  //     return false;
  // }

}

// ============================================================================
void Queue::printQueue() const {
  std::cout << "--------------------" << std::endl;
  std::cout << "Queue Size: " << mVertexQueue.size() << std::endl;
  std::cout << "--------------------" << std::endl;
  for (auto iterQ = mVertexQueue.begin(); iterQ != mVertexQueue.end(); ++iterQ) {
    auto pair = *iterQ;
    std::cout << "Vertex: " << pair.first << " "
              << "Cost: " << pair.second.first << ", "
              << pair.second.second << std::endl;
  }
  std::cout << "--------------------" << std::endl;
}

} // namespace datastructures
} // namespace lgls

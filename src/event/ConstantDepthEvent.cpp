#include "lgls/event/ConstantDepthEvent.hpp"

namespace lgls {
namespace event {

using lgls::datastructures::Edge;
using lgls::datastructures::EvaluationStatus;
using lgls::datastructures::Graph;
using lgls::datastructures::Vertex;
using lgls::datastructures::Path;

//==============================================================================
ConstantDepthEvent::ConstantDepthEvent(std::size_t depth) : mDepthThreshold(depth) {
  // Do nothing.
}

//==============================================================================
bool ConstantDepthEvent::isTriggered(const Vertex& vertex) {
  if (vertex == mTargetVertex) {
    return true;
  }

  if (getDepth(vertex) == mDepthThreshold) {
    return true;
  }
  return false;
}

//==============================================================================
bool ConstantDepthEvent::isTriggered(const Path& path) {

  // return if the end vertex is the target vertex Note that obtainPath reverts
  if (path[0] == mTargetVertex) {
    return true;
  }

  // count the unevaluated edges
  size_t unevaluatedEdges = 0 ;
  for (std::size_t i = path.size() - 1; i > 0; --i) {

    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(path[i], path[i-1], (*mGraph));
    if ((*mGraph)[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
        unevaluatedEdges++;
    }

  } // End for loop

  if (unevaluatedEdges == mDepthThreshold) {
    return true;
  }
  return false;
}

//==============================================================================
void ConstantDepthEvent::updateVertexProperties(Vertex& vertex) {
  // Do nothing
}

//==============================================================================
std::size_t ConstantDepthEvent::getDepth(const Vertex& vertex) {

  size_t unevaluatedEdges = 0 ;

  Vertex currentVertex = vertex;
  // EXTREMELY IMPORTANT : DO NOT CALL auto FOR ACCESSING GRAPH, IT WILL COPY IT.
  // Access the graph.
  // auto graph = *mGraph;
  while (currentVertex!= mSourceVertex)
  {
    assert((*mGraph)[currentVertex].hasParent());
    Vertex parentVertex = (*mGraph)[currentVertex].getParent();
    Edge uv;
    bool edgeExists;
    boost::tie(uv, edgeExists) = edge(parentVertex, currentVertex, (*mGraph));
    if ((*mGraph)[uv].getEvaluationStatus() == EvaluationStatus::NotEvaluated)
    {
        unevaluatedEdges++;
    }
    currentVertex = parentVertex;
  }

  return unevaluatedEdges;
}




} // namespace event
} // namespace lgls

/* Authors: Aditya Vamsikrishna Mandalika */
/* Modified by Jaein Lim */

#include "lgls/datastructures/Graph.hpp"

namespace lgls {
namespace datastructures {

// ============================================================================
void VertexProperties::setState(StatePtr state) {
  mState = state;
}

// ============================================================================
StatePtr VertexProperties::getState() {
  return mState;
}

// ============================================================================
void VertexProperties::setCostToCome(double cost) {
  mCostToCome = cost;
}

// ============================================================================
double VertexProperties::getCostToCome() {
  return mCostToCome;
}

// ============================================================================
void VertexProperties::setRHS(double cost) {
  mRHS = cost;
}

// ============================================================================
double VertexProperties::getRHS() {
  return mRHS;
}

// ============================================================================
void VertexProperties::setHeuristic(double heuristic) {
  mHeuristic = heuristic;
}

// ============================================================================
double VertexProperties::getHeuristic() {
  return mHeuristic;
}

// ============================================================================
void VertexProperties::setParent(Vertex parent) {
  mParent = parent;
}

// ============================================================================
Vertex VertexProperties::getParent() {
  return mParent;
}

// ============================================================================
bool VertexProperties::hasParent() {
  return mParent != boost::graph_traits<BasicGraph>::null_vertex();
}

// ============================================================================
void VertexProperties::removeParent( ) {
  mParent = boost::graph_traits<BasicGraph>::null_vertex();
}

// ============================================================================
void VertexProperties::setGpi(double gpi) {
  mGpi = gpi;
}

// ============================================================================
double VertexProperties::getGpi() {
  return mGpi;
}

// ============================================================================
void VertexProperties::setPi(const std::vector<Vertex>& pi)
{
  mPi = pi;
}

// ============================================================================
std::vector<Vertex> VertexProperties::getPi()
{
  return mPi;
}

// ============================================================================
void VertexProperties::removePi() {
  mPi = {};
}

// ============================================================================
void VertexProperties::setEvaluationStatus(EvaluationStatus evaluationStatus) {
  mEvaluationStatus = evaluationStatus;
}

// ============================================================================
EvaluationStatus VertexProperties::getEvaluationStatus() {
  return mEvaluationStatus;
}

// ============================================================================
void VertexProperties::setCollisionStatus(CollisionStatus status) {
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus VertexProperties::getCollisionStatus() {
  return mCollisionStatus;
}


// ============================================================================
void EdgeProperties::setLength(double length) {
  mLength = length;
}

// ============================================================================
double EdgeProperties::getLength() {
  return mLength;
}

// ============================================================================
void EdgeProperties::setValue(double value) {
  mValue = value;
}

// ============================================================================
double EdgeProperties::getValue() {
  if (mEvaluationStatus)
    return mValue;
  else
    return mLength;
}

// ============================================================================
double EdgeProperties::getValue(double inflation=1) {
  if (mEvaluationStatus)
    return mValue;
  else
    return inflation*mLength;
}

// ============================================================================
void EdgeProperties::setEvaluationStatus(EvaluationStatus evaluationStatus) {
  mEvaluationStatus = evaluationStatus;
}

// ============================================================================
EvaluationStatus EdgeProperties::getEvaluationStatus() {
  return mEvaluationStatus;
}

// ============================================================================
void EdgeProperties::setCollisionStatus(CollisionStatus status) {
  mCollisionStatus = status;
}

// ============================================================================
CollisionStatus EdgeProperties::getCollisionStatus() {
  return mCollisionStatus;
}


} // namespace datastructures
} // namespace lgls

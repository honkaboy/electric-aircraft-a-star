#include "node.h"

#include <limits>

namespace {
using Airports::AirportID;
}  // namespace

Node::Node()
    : estimated_cost(std::numeric_limits<double>::infinity()),
      geographic_path({}),
      charge_amounts({}) {}

Node::Node(double cost, std::vector<AirportID> path,
           std::unordered_map<AirportID, double> charge)
    : estimated_cost(cost), geographic_path(path), charge_amounts(charge) {}

bool Node::operator<(const Node& rhs) const {
  return this->estimated_cost < rhs.estimated_cost;
}
bool Node::operator>(const Node& rhs) const {
  return this->estimated_cost > rhs.estimated_cost;
}


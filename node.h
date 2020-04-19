#pragma once

#include <unordered_map>
#include <vector>

#include "common.h"
#include "airports.h"

/// A class to store a node of the graph space defined in graph.h. See graph.h for
/// details!
// TODO make all members const
struct Node {
  Node();
  Node(double cost, std::vector<Airports::AirportID> path,
       std::unordered_map<Airports::AirportID, double> charge);

  /// < and > operators defined so that the nodes can be sorted by estimated total cost.
  bool operator<(const Node& rhs) const;
  bool operator>(const Node& rhs) const;

  // \brief Estimated total cost to goal if full path goes through geographic_path first.
  double estimated_cost;

  /// \brief An ordered list of nodes that defines the partial or full geographic path for
  /// this Node.
  std::vector<Airports::AirportID> geographic_path;

  /// The amount of charge that the aircraft must obtain at each airport along the minimum
  /// time path traversing geographic_path.
  std::unordered_map<Airports::AirportID, double> charge_amounts;
};

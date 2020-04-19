#pragma once

#include "common.h"
#include "node.h"

/// \brief A solver for the electric aircraft routing problem.
///
/// The class solves the problem by running class A* search over an abstract graph space,
/// as defined below:
///  - Vertex (called Node in this code): A partial or full geographic path from an origin
///  map airport to a goal map airport through 0 or more intermediate airports.
///  - Edge: (directional) The extra (estimated) length of time required to traverse Child
///  as compared to Parent.
///  That is: child nodes of a parent have the same geographic path plus one extra leg.
///
/// By defining
///  (1) a function that computes the minimum time to traverse a geographic path, given
// aircraft and map charging characteristics
///  (2) a heuristic for calculating the remaining cost to go
/// The remainder of the algorithm is classic A* graph search.
class Graph {
 public:
  Graph(const std::vector<Airports::AirportData>& network);

  /// \brief Determine the great circle distance between two airports of the airport network.
  double GreatCircleDistance(const Airports::AirportID id_a,
                             const Airports::AirportID id_b) const;

  /// \brief Find the set of nearby airports not in a given exclusion list.
  /// \note: exclusion_list is intentionally passed by value so that it can be sorted in
  /// place.
  /// TODO improve exclusion calculation by using HashSets instead of lists?
  std::vector<Airports::AirportID> GetNearbyAirportDatas(
      const Airports::AirportID origin_airport,
      std::vector<Airports::AirportID> exclusion_list) const;

  /// \brief Compute the minimum cost (time), given max aircraft range and charge
  /// requirements, to traverse a given geographic path across the map. Also computes the
  /// amounts charged at each charged station along \p geographic_path.
  void ComputeMinCostAlongPath(
      const std::vector<Airports::AirportID>& geographic_path,
      double& min_cost_along_path,
      std::unordered_map<Airports::AirportID, double>& charge_amounts) const;

  /// \brief Compute an estimated cost (time), given map charge characteristics,
  /// required to reach a final airport from an intermediate airport.
  ///
  /// This heuristic is not admissible, but seems to work well in practice. More notes
  /// in the implementation.
  double ComputeEstimatedCostToGo(const Airports::AirportID intermediate_airport,
                                  const Airports::AirportID goal_airport) const;

  /// \brief Compute the estimated total cost to reach a \p goal airport, given that we
  /// first traverse \p geographic_path. Also computes the amounts charged at each charged
  /// station along \p geographic_path.
  void ComputeEstimatedTotalCost(
      const std::vector<Airports::AirportID>& geographic_path,
      const Airports::AirportID goal_airport, double& estimated_total_cost,
      std::unordered_map<Airports::AirportID, double>& charge_amounts) const;

  /// \brief Compute the children node of a given \p parent within the graph.
  std::vector<Node> GetChildren(const Node& parent,
                                const Airports::AirportID goal_airport) const;

  /// \brief Find the shortest (in time) path from \p initial_airport_name to \p
  /// goal_airport_name, given the map and the aircraft and airport constraints.
  ///
  /// If no solution exists or if the node names are invalid, return an empty Node.
  Node Solve(const std::string& initial_airport_name,
             const std::string& goal_airport_name);

  /// \brief Find the shortest (in time) path from \p initial_airport_name to \p
  /// goal_airport_name, given the map and the aircraft and airport constraints.
  Node Solve(const Airports::AirportID initial_airport,
             const Airports::AirportID goal_airport);

 private:
  // Maximum number of graph nodes to expand in searching for the solution.
  static constexpr size_t kMaxGraphNodes = 10e6;

  /// The network of airports available.
  const std::vector<Airports::AirportData>& kNetwork;

  /// The distances between airports.
  std::vector<std::vector<double>> airport_distance_;

  /// For each airport, a list of nearby airports, sorted by index.
  std::vector<std::vector<Airports::AirportID>> nearby_airports_;

  /// The minimum charge rate in the network.
  double minimum_charge_rate_;
};

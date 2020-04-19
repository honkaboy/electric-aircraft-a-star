#include "graph.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace {
using Airports::AirportData;
using Airports::AirportID;
const size_t kNumAirports = Airports::kAirports.size();
}  // namespace

Graph::Graph(const std::vector<AirportData>& network) : kNetwork(network) {
  // Create airport distance matrix and nearby airport maps.
  // Allocate airport_distance map.
  for (size_t i = 0; i < kNumAirports; ++i) {
    airport_distance_.emplace_back(kNumAirports,
                                   std::numeric_limits<double>::signaling_NaN());
  }
  // Note: Site distance matrix should be symmetric with diagonal == 0, so we only need
  // to calculated the upper triangle and copy to the lower.
  for (size_t i = 0; i < kNumAirports; ++i) {
    airport_distance_[i][i] = 0.0;
    for (size_t j = i + 1; j < kNumAirports; ++j) {
      // Site distance matrix.
      const double dist = GreatCircleDistance(i, j);
      airport_distance_[i][j] = dist;
      airport_distance_[j][i] = dist;
    }
  }

  // Nearby airport maps must be sorted ascending anyway, so we create them in a separate
  // nested for loop.
  // Allocate nearby_airports_
  for (size_t i = 0; i < kNumAirports; ++i) {
    std::vector<AirportID> row;
    for (size_t j = 0; j < kNumAirports; ++j) {
      const double dist = airport_distance_[i][j];
      if (dist <= kAircraftRange) {
        row.push_back(j);
      }
    }
    nearby_airports_.push_back(row);
  }

  // Find the minimum charge rate among all airports.
  minimum_charge_rate_ = std::numeric_limits<double>::infinity();
  for (const AirportData& airport : kNetwork) {
    minimum_charge_rate_ = std::min(minimum_charge_rate_, airport.rate * kKPH2MPS);
  }
}

double Graph::GreatCircleDistance(const AirportID id_a, const AirportID id_b) const {
  const AirportData& airport_a = kNetwork[id_a];
  const AirportData& airport_b = kNetwork[id_b];

  return GreatCircleDistance_f(airport_a.lat, airport_a.lon, airport_b.lat,
                               airport_b.lon);
}

std::vector<AirportID> Graph::GetNearbyAirportDatas(
    const AirportID origin_airport, std::vector<AirportID> exclusion_list) const {
  // All airports geographically near origin_airport.
  const std::vector<AirportID>& nearby_airports = nearby_airports_[origin_airport];
  std::vector<AirportID>::iterator it;
  std::vector<AirportID> allowed_nearby_airports(nearby_airports.size());

  // Both lists must be sorted for the set difference computation.
  if (!std::is_sorted(nearby_airports.begin(), nearby_airports.end())) {
    throw std::logic_error("nearby_airports not sorted!");
  }
  std::sort(exclusion_list.begin(), exclusion_list.end());

  // Perform the set difference and return the result.
  it = std::set_difference(nearby_airports.begin(), nearby_airports.end(),
                           exclusion_list.begin(), exclusion_list.end(),
                           allowed_nearby_airports.begin());
  allowed_nearby_airports.resize(it - allowed_nearby_airports.begin());

  return allowed_nearby_airports;
}

void Graph::ComputeMinCostAlongPath(
    const std::vector<AirportID>& geographic_path, double& min_cost_along_path,
    std::unordered_map<AirportID, double>& charge_amounts) const {
  const AirportID origin = geographic_path[0];

  // distance(i) is distance from airport i to airport i+1.
  std::unordered_map<AirportID, double> distances;
  for (size_t i = 0; i < (geographic_path.size()) - 1; ++i) {
    const AirportID airport_a = geographic_path[i];
    const AirportID airport_b = geographic_path[i + 1];
    distances[airport_a] = airport_distance_[airport_a][airport_b];
  }

  // Create list of airports sorted ascending by charge rate.
  // We only charge at intermediate airports, so throw out first and last.
  std::vector<AirportID> worst_airports(geographic_path.begin() + 1,
                                        geographic_path.end() - 1);
  std::sort(worst_airports.begin(), worst_airports.end(),
            [this](AirportID a, AirportID b) {
              return this->kNetwork[a].rate < this->kNetwork[b].rate;
            });

  // Determine how much "slack" charge we have available to allocate across the route
  // after we've arrived at the first airport, assuming we start from origin with full
  // charge. Allocate the slack to the worst airports along the route, arriving at the
  // final known location of the path with 0 charge.
  double slack = kAircraftRange - distances[origin];
  for (const AirportID airport : worst_airports) {
    const double remaining_slack = slack - distances[airport];
    if (remaining_slack <= 0.0) {
      charge_amounts[airport] = distances[airport] - slack;
      slack = 0;
    } else {
      charge_amounts[airport] = 0.0;
      slack = remaining_slack;
    }
  }

  // Add up total cost (in time).
  // Cost to traverse from origin to first leg. No charging at first airport, of course.
  double cost = 0.0;
  cost += distances[origin] / kAircraftSpeed;
  // Cost to traverse each remaining leg, including the charge time at the origin
  // airport of each leg.
  for (size_t i = 1; i < (geographic_path.size() - 1); ++i) {
    const AirportID airport_id = geographic_path[i];
    // rate in km / hr, convert to SI.
    const double rate_kph = kNetwork[airport_id].rate;
    const double rate = rate_kph * kKPH2MPS;
    cost += charge_amounts[airport_id] / rate;
    cost += distances[airport_id] / kAircraftSpeed;
  }

  min_cost_along_path = cost;
  return;
}

double Graph::ComputeEstimatedCostToGo(const AirportID intermediate_airport,
                                       const AirportID goal_airport) const {
  /// Estimate the cost to go by assuming there are lots of airports all in a
  /// perfect line to the goal airport, and that they all charge at the maximum possible
  /// rate.
  ///
  /// Note: In theory, we would want the estimated cost to go to be an underestimate,
  /// which would mean using the *maximum* charging speed. However, this makes the search
  /// space explode! (It's a large network!) I experimented using the average, minimal,
  /// and maximal charge rates. The only one that converged fast enough to be reasonable
  /// was using the maximal charge rate.  It makes sense, of course, that this would find
  /// a solution faster, being the most optimistic, but it's not optimal.
  ///
  /// TODO It would be nice to find an admissible heuristic that is very good to help our
  /// changes of finding the optimal solution.
  const double dist_to_go = airport_distance_[intermediate_airport][goal_airport];
  const double est_charge_time_to_go = dist_to_go / minimum_charge_rate_;
  const double est_travel_time_to_go = dist_to_go / kAircraftSpeed;
  const double cost = est_charge_time_to_go + est_travel_time_to_go;

  return cost;
}

void Graph::ComputeEstimatedTotalCost(
    const std::vector<AirportID>& geographic_path, const AirportID goal_airport,
    double& estimated_total_cost,
    std::unordered_map<AirportID, double>& charge_amounts) const {
  double cost_so_far = 0.0;
  ComputeMinCostAlongPath(geographic_path, cost_so_far, charge_amounts);
  const double estimated_cost_to_go =
      ComputeEstimatedCostToGo(geographic_path.back(), goal_airport);
  estimated_total_cost = cost_so_far + estimated_cost_to_go;
  return;
}

std::vector<Node> Graph::GetChildren(const Node& parent,
                                     const AirportID goal_airport) const {
  std::vector<Node> children;

  // Generate valid children by adding valid nearby airports to the end of the
  // parent node's geographic path. Valid airports are those that aren't already found
  // in the parent's path.
  const std::vector<AirportID> new_nearby_airports =
      GetNearbyAirportDatas(parent.geographic_path.back(), parent.geographic_path);

  // Create a child node of parent for each qualifying airport.
  for (const AirportID airport : new_nearby_airports) {
    // Compute child's geographic path.
    std::vector<AirportID> child_geographic_path = parent.geographic_path;
    child_geographic_path.push_back(airport);

    // Compute the estimated total distance across child's geographic path.
    double child_estimated_total_cost = 0.0;
    std::unordered_map<AirportID, double> charge_amounts;
    ComputeEstimatedTotalCost(child_geographic_path, goal_airport,
                              child_estimated_total_cost, charge_amounts);

    children.emplace_back(child_estimated_total_cost, child_geographic_path,
                          charge_amounts);
  }

  return children;
}

Node Graph::Solve(const std::string& initial_airport_name,
                  const std::string& goal_airport_name) {
  AirportID initial_airport = std::numeric_limits<AirportID>::max();
  AirportID goal_airport = std::numeric_limits<AirportID>::max();
  for (size_t i = 0; i < kNumAirports; ++i) {
    if (kNetwork[i].name == initial_airport_name) {
      initial_airport = i;
    }
    if (kNetwork[i].name == goal_airport_name) {
      goal_airport = i;
    }
  }

  /// TODO Do something better error-handling wise than just return an empty node if the
  /// user passed a bad set of inputs. As such, this still works prevents unexpected
  /// behavior. The program will just print "no solution found".
  if (initial_airport == std::numeric_limits<AirportID>::max()) {
    return Node();
  }
  if (goal_airport == std::numeric_limits<AirportID>::max()) {
    return Node();
  }

  return Solve(initial_airport, goal_airport);
}

Node Graph::Solve(const AirportID initial_airport, const AirportID goal_airport) {
  // TODO Add input error checking - make sure the airport IDs are in fact valid.

  if (initial_airport == goal_airport) {
    // We're already there. Solution is known for this case.
    return Node(0.0, {initial_airport}, {});
  }
  // Create (ascending) priority queue, prioritized by each node's expected total cost.
  std::priority_queue<Node, std::deque<Node>, std::greater<Node>> queue;
  // The first node to be expanded consists of a path with a single node: our starting
  // spot.
  const Node start(std::numeric_limits<double>::infinity(), {initial_airport}, {});
  queue.push(start);

  size_t expand_count = 0;

  // TODO Improve error handling. Indicate whether we failed to
  // find a solution because we maxed out the number of nodes to expand (i.e. ran out of
  // time) or if we exhaustively searched the map and the solution simply does not exist.
  Node solution = start;
  while (!queue.empty() && expand_count++ < kMaxGraphNodes) {
    // Examine the current best node.
    const Node current_node = queue.top();
    queue.pop();

    if (current_node.geographic_path.back() == goal_airport) {
      // If we've reached the goal location, stop searching the graph and return the
      // solution.
      solution = current_node;
      break;
    } else {
      // Otherwise, continue to search the graph by adding child nodes of the
      // current node to the search queue.
      std::vector<Node> children = GetChildren(current_node, goal_airport);
      for (const Node& child : children) {
        // NOTE: This inserts in priority order because Node has operator< defined.
        queue.push(child);
      }
    }
  }

  /// TODO(nhonka) Add a quick sanity checker here that verifies path satisfiability:
  /// - All legs along the route are less than aircraft range.
  /// - Aircraft charge never falls below 0 given charging schedule.

  return solution;
}

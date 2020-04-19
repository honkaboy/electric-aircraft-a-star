#include <iostream>

#include "graph.h"
#include "airports.h"
#include "node.h"

namespace {
using Airports::kAirports;
using Airports::AirportID;
using Airports::AirportData;
}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cout << "Error: requires initial and final airport names" << std::endl;
    return -1;
  }

  std::string initial_airport_name = argv[1];
  std::string goal_airport_name = argv[2];

  // Construct the graph in the search space and find the solution.
  Graph g(Airports::kAirports);
  Node solution = g.Solve(initial_airport_name, goal_airport_name);

  // Print out data about the solution.
  if (std::isfinite(solution.estimated_cost)) {
    const std::vector<AirportID>& path = solution.geographic_path;
    for (size_t i = 0; i < path.size(); ++i) {
      const AirportID airport_idx = path[i];
      const AirportData& airport = kAirports[airport_idx];
      if (i == (path.size() - 1)) {
        // Last station
        std::cout << airport.name << std::endl;
      } else if (i == 0) {
        // First station.
        std::cout << airport.name << " -- charge time: 0" << std::endl;
      } else {
        // Intermediate station.
        // Convert m -> km for print
        const double charge_time_hr =
            solution.charge_amounts[airport_idx] / airport.rate / 1000.0;
        std::cout << airport.name << " -- charge time: " << charge_time_hr << " hr" << std::endl;
      }
    }
    std::cout << std::endl;
  } else {
    std::cout << "no solution found" << std::endl;
  }

  return 0;
}

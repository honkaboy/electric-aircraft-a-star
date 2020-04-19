# Electric Aircraft Minimum Travel Time Search
## Project Overview
This project just exists to be a code sample. It contains a search algorithm to plan a flight route through the United States for an electric aircraft such that:

1. The aircraft never runs out of energy.
2. The amount of time to traverse the whole path is minimized.

## Problem setup:
1. The aircraft may stop at any airport it can reach with its current battery life. For the sake of simplicity, I assume distance between airports are simply great circle distances between cities. The aircraft starts charging immediately when it arrives.
2. The aircraft starts with a full charge and enjoys a constant rate of travel between airports.

## Solution:
This project implements A* search over an abstract graph space, as defined below:
* Vertex (called Node in this code): A partial or full geographic path from an origin airport to a goal airpot through 0 or more intermediate airports.
* Edge: (directional) The extra (estimated) length of time required to traverse Child
  as compared to Parent.
  That is: child nodes of a parent have the same geographic path plus one extra leg.

 By defining
 1. A function `Graph::ComputeMinCostAlongPath` that computes the minimum cost (time) to traverse a geographic path, given aircraft and map charging characteristics.
 2. A heuristic `Graph::ComputeEstimatedCostToGo` for calculating the remaining cost (time) to goal.
  
the nodes and edges can be computed, and problem then reduces to classic A* search! 

## Usage
Specify the two airports / cities to solve for a path between as follows:

    ./solver Kansas_City_MO Milpitas_CA

    Kansas_City_MO -- charge time: 0
    Glasco_KS -- charge time: 2.02533 hr
    Collyer_KS -- charge time: 1.88029 hr
    Coolidge_KS -- charge time: 3.21461 hr
    Lincoln_Park_CO -- charge time: 2.67178 hr
    Nucla_CO -- charge time: 3.12239 hr
    Escalante_UT -- charge time: 1.69204 hr
    Central_UT -- charge time: 1.98343 hr
    Pahrump_NV -- charge time: 3.32334 hr
    Sultana_CA -- charge time: 2.42898 hr
    Milpitas_CA

A list of available airports can be found in airports.h.

## Data
City data used to generate airport map downloaded from https://simplemaps.com/data/us-cities. License included in `city_data/license.txt`.

#pragma once

#include <cmath>
#include "airports.h"

// All units SI unless otherwise specified.
constexpr double kEarthRadius = 6371000.;
constexpr double kAircraftSpeed_kph = 120.;
constexpr double kKPH2MPS = 1000. / 3600.;
constexpr double kSEC2HR = 1. / 3600.;
constexpr double kAircraftSpeed = kAircraftSpeed_kph * kKPH2MPS;
constexpr double kAircraftRange = 300000.;

/// \brief Convert degrees to radians, obviously.
double Deg2Rad(double deg);

/// \brief Calculate the great circle distance between two locations on the globe.
double GreatCircleDistance_f(const double latitude1, const double longitude1,
                             const double latitude2, const double longitude2);

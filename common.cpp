#include "common.h"

double Deg2Rad(double deg) { return (deg * M_PI / 180.0); }

// Copied from https://gist.github.com/ed-flanagan/e6dc6b8d3383ef5a354a
double GreatCircleDistance_f(const double latitude1, const double longitude1,
                             const double latitude2, const double longitude2) {
  const double lat1 = Deg2Rad(latitude1);
  const double lon1 = Deg2Rad(longitude1);
  const double lat2 = Deg2Rad(latitude2);
  const double lon2 = Deg2Rad(longitude2);

  const double d_lon = std::abs(lon1 - lon2);

  // Numerator
  const double a = std::pow(std::cos(lat2) * std::sin(d_lon), 2);

  const double b = std::cos(lat1) * std::sin(lat2);
  const double c = std::sin(lat1) * std::cos(lat2) * std::cos(d_lon);
  const double d = std::pow(b - c, 2);

  const double e = sqrt(a + d);

  // Denominator
  const double f = std::sin(lat1) * std::sin(lat2);
  const double g = std::cos(lat1) * std::cos(lat2) * std::cos(d_lon);

  const double h = f + g;

  const double d_sigma = std::atan2(e, h);

  return kEarthRadius * d_sigma;
}

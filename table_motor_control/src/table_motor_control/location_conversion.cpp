
#include "location_conversion.hpp"

namespace location_conversion
{
  double getLinearPosition(double pos, std::pair<double,double> range, std::pair<double,double> joint_limits)
  {
    double third_distance = range.second / 3.0;
    double output = 0;

    if(pos > third_distance && pos <= 2*third_distance) pos -= third_distance;
    else if(pos > 2*third_distance) pos -= (2*third_distance);

    return map_ranges(pos, range.first, range.first + third_distance, joint_limits.first, joint_limits.second);
  }

  double map_ranges(double input, double from_min, double from_max, double to_min, double to_max)
  {
    double ratio = (to_max - to_min) / (from_max - from_min);
    return to_min + ratio * (input - from_min);
  }
}

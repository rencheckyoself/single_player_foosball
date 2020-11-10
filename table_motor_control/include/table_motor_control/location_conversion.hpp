/// \file
/// \brief a library to convert the ball positions into motor commands
#ifndef LOC_CONV_INCLUDE_GUARD_HPP
#define LOC_CONV_INCLUDE_GUARD_HPP

#include <utility>

namespace location_conversion
{

  /// \brief Uses the y coordinate of the ball to determine the linear position of the rod
  /// \param pos the y coordinate of the ball/the position to move a player to
  /// \param range the min (first) and max (second) defining the range of possible y coordinates
  /// \param joint_limits the upper and lower limit for joint positions
  /// \returns the value to move the linear joint to
  double getLinearPosition(double pos, std::pair<double,double> range, std::pair<double,double> joint_limits);

  /// \brief Map a value from one range to another
  /// \param input the value to map
  /// \param from_min the minimum of the range you are mapping from
  /// \param from_max the maximum of the range you are mapping from
  /// \param to_min the minimum of the range you are mapping to
  /// \param to_max the maximum of the range you are mapping to
  /// \returns the remapped input value
  /// \TODO: add bounds checking on the input value
  double map_ranges(double input, double from_min, double from_max, double to_min, double to_max);


}
#endif
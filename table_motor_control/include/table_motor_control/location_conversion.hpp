/// \file
/// \brief a library to convert the ball positions into motor commands
#ifndef LOC_CONV_INCLUDE_GUARD_HPP
#define LOC_CONV_INCLUDE_GUARD_HPP

#include <utility>
#include <algorithm>
#include <cmath>
#include "tic_server.hpp"

namespace table
{

  struct RodState
  {
    double lin_pos; ///< rotational joint position
    double rot_pos; ///< linear joint position
  };

  class TableController
  {
  public:

  private:

    RodState fwd_rod_joints; ///< forward rod joint states
    RodState def_rod_joints; ///< defense rod joint states

    int fwd_rot_steps; ///< forward rod rotational target stepper position
    int fwd_lin_steps; ///< forward rod linear target stepper position

    int def_rot_steps; ///< defense rod rotational target stepper position
    int def_lin_steps; ///< defense rod linear target stepper position

    tic_server::TicCtrlr fwd_rot; ///< attacking rotational rod controller
    tic_server::TicCtrlr fwd_lin; ///< attacking linear rod controller
    tic_server::TicCtrlr def_rot; ///< defensive rotational rod controller
    tic_server::TicCtrlr def_lin; ///< defensive linear rod controller
  };

}

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

  /// \brief Use the x coordinate of the ball to determine the angular position of the rod
  /// \param pos the x coordinate of the ball
  /// \param thresh the threshold to send a kick command
  /// \param rod_x_coord the x coordinate the ball needs to be at to optimally kick the ball
  /// \returns an anlge in radians to move the rod to
  /// \TODO: Account for ball velocity or recalculate thresh to account for the veloctiy
  double getAngularPosition(double pos, double thresh, double rod_x_coord);

  /// \brief reads in the joint state message and converts it to the proper stepper command
  void jointMsg_to_Steppers();

}
#endif

//
// NOTE: The Tic's control mode must be "Serial / I2C / USB".

/// \file
/// \brief Node to subsribe to commands and send them to the Tic's. Also offers services for manual control.
/// PARAMETERS:
/// PUBLISHES:
///
/// SUBSCRIBES:
///
/// SERIVCES:

#include <ros/ros.h>

#include <iostream>
#include <tic.hpp>

#include "tic_server.hpp"

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "tic_cmd");
  ros::NodeHandle np("~");

  std::string fwd_rot_sn, fwd_lin_sn;

  np.getParam("fwd_rot/serial_number", fwd_rot_sn);
  np.getParam("fwd_lin/serial_number", fwd_lin_sn);

  ROS_INFO_STREAM("TICCMD: Forward Rotation ID: " << fwd_rot_sn);
  ROS_INFO_STREAM("TICCMD: Forward Linear ID: " << fwd_lin_sn);

  tic_server::TicCtrlr fwd_rot(fwd_rot_sn);
  // tic_server::TicCtrlr fwd_lin(fwd_lin_sn);

  fwd_rot.get_current_pos();
  // fwd_lin.get_current_pos();

  // fwd_rot.display_settings();

  fwd_rot.import_settings();

  ros::spin();

  return 0;
}

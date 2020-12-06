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

  std::string fwd_rot_sn, fwd_lin_sn, def_rot_sn, def_lin_sn;
  std::string fwd_rot_nickname, fwd_lin_nickname, def_rot_nickname, def_lin_nickname;

  np.getParam("fwd_rot/serial_number", fwd_rot_sn);
  np.getParam("fwd_rot/nickname", fwd_rot_nickname);

  np.getParam("fwd_lin/serial_number", fwd_lin_sn);
  np.getParam("fwd_lin/nickname", fwd_lin_nickname);

  np.getParam("def_rot/serial_number", def_rot_sn);
  np.getParam("def_rot/nickname", def_rot_nickname);

  np.getParam("def_lin/serial_number", def_lin_sn);
  np.getParam("def_lin/nickname", def_lin_nickname);

  ROS_INFO_STREAM("TICCMD: Forward Rotation ID: " << fwd_rot_sn);
  ROS_INFO_STREAM("TICCMD: Forward Linear ID: " << fwd_lin_sn);
  ROS_INFO_STREAM("TICCMD: Defense Rotation ID: " << def_rot_sn);
  ROS_INFO_STREAM("TICCMD: Defense Linear ID: " << def_lin_sn);

  ROS_INFO_STREAM("TICCMD: Forward Rotation Name: " << fwd_rot_nickname);
  ROS_INFO_STREAM("TICCMD: Forward Linear Name: " << fwd_lin_nickname);
  ROS_INFO_STREAM("TICCMD: Defense Rotation Name: " << def_rot_nickname);
  ROS_INFO_STREAM("TICCMD: Defense Linear Name: " << def_lin_nickname);

  tic_server::TicCtrlr fwd_rot(fwd_rot_sn, fwd_rot_nickname);
  tic_server::TicCtrlr fwd_lin(fwd_lin_sn, fwd_lin_nickname);

  tic_server::TicCtrlr def_rot(def_rot_sn, def_rot_nickname);
  tic_server::TicCtrlr def_lin(def_lin_sn, def_lin_nickname);

  fwd_rot.offer_services();
  fwd_lin.offer_services();
  def_rot.offer_services();
  def_lin.offer_services();

  ros::spin();

  return 0;
}

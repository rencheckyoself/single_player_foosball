/// \file
/// \brief Node to subsribe to commands and send them to the Tic's. Also offers services for manual control.
/// PARAMETERS:
/// PUBLISHES:
///
/// SUBSCRIBES:
///
/// SERIVCES:

#include <ros/ros.h>

#include "ball_prediction.hpp"
#include "tic_server.hpp"

double map_ranges(double input, double from_min, double from_max, double to_min, double to_max)
{
  double ratio = (to_max - to_min) / (from_max - from_min);
  return to_min + ratio * (input - from_min);
}

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_ball");

  ros::NodeHandle np("~");
  ros::NodeHandle n;

  std::string def_rot_sn, def_lin_sn;
  std::string def_rot_nickname, def_lin_nickname;

  np.getParam("def_rot/serial_number", def_rot_sn);
  np.getParam("def_rot/nickname", def_rot_nickname);

  np.getParam("def_lin/serial_number", def_lin_sn);
  np.getParam("def_lin/nickname", def_lin_nickname);

  ROS_INFO_STREAM("TICCMD: Defensive Rotation ID: " << def_rot_sn);
  ROS_INFO_STREAM("TICCMD: Defensive Linear ID: " << def_lin_sn);

  ROS_INFO_STREAM("TICCMD: Defensive Rotation Name: " << def_rot_nickname);
  ROS_INFO_STREAM("TICCMD: Defensive Linear Name: " << def_lin_nickname);

  tic_server::TicCtrlr def_rot(def_rot_sn, def_rot_nickname);
  tic_server::TicCtrlr def_lin(def_lin_sn, def_lin_nickname);

  tracking::BallTracker foosball(0);

  ros::Rate r(100);

  foosball.testExtrinsicResults();

  while(ros::ok())
  {
    // Get the position of the ball
    cv::Point3d pos = foosball.getWorldPosition();

    ROS_INFO_STREAM("Ball Position: " << pos);



    ros::spinOnce();

    r.sleep();
  }


  return 0;
}

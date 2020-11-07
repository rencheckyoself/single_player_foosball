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

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_ball");
  ros::NodeHandle np("~");
  ros::NodeHandle n;

  // Get the position of the ball

  // identify the third of the field the ball is in

  // convert ball pixel position to stepper position

  // send linear stepper command



  ros::spin();

  return 0;
}

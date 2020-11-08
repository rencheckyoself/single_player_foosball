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

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_ball");

  tracking::BallTracker foosball(0);

  ros::Rate r(100);

  foosball.testExtrinsicResults();

  // ros::spin();

  while(ros::ok())
  {
    // Get the position of the ball
    cv::Point3d pos = foosball.getWorldPosition();

    // ROS_INFO_STREAM("Ball Position: " << pos);

    ros::spinOnce();

    r.sleep();
  }


  return 0;
}

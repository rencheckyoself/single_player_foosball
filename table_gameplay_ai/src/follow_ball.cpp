/// \file
/// \brief Node to subsribe to commands and send them to the Tic's. Also offers services for manual control.
/// PARAMETERS:
/// PUBLISHES:
///
/// SUBSCRIBES:
///
/// SERIVCES:

#include <utility>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>

#include "ball_prediction.hpp"
#include "table_motor_control/tic_server.hpp"
#include "table_motor_control/location_conversion.hpp"

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_ball");

  ros::NodeHandle np("~");
  ros::NodeHandle n;

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

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

  // tic_server::TicCtrlr def_rot(def_rot_sn, def_rot_nickname);
  // tic_server::TicCtrlr def_lin(def_lin_sn, def_lin_nickname);

  tracking::BallTracker foosball(0);

  ros::Rate r(100);

  foosball.testExtrinsicResults();

  sensor_msgs::JointState joint_msg;

  std::vector<std::string> joint_names = {"white_attack_rot_joint", "white_attack_lin_joint", "white_goalie_rot_joint", "white_goalie_lin_joint", "grey_attack_rot_joint", "grey_attack_lin_joint", "grey_goalie_rot_joint", "grey_goalie_lin_joint"};
  joint_msg.name = joint_names;

  // std::pair<double,double> xrange = foosball.getXRange();
  std::pair<double,double> yrange = foosball.getYRange();

  while(ros::ok())
  {
    std::vector<double> joint_vals(joint_names.size(), 0);

    // Get the position of the ball
    cv::Point3d pos = foosball.getWorldPosition();

    // ROS_INFO_STREAM(pos);

    double def_lin_pos = location_conversion::getLinearPosition(pos.y, yrange, std::make_pair<double,double>(-0.045, 0.045));
    double def_rot_pos = location_conversion::getAngularPosition(pos.x, 0.085, 0.010);

    joint_vals.at(3) = def_lin_pos;
    joint_vals.at(2) = def_rot_pos;

    joint_msg.header.stamp = ros::Time::now();
    joint_msg.position = joint_vals;

    joint_pub.publish(joint_msg);

    ros::spinOnce();

    r.sleep();
  }


  return 0;
}

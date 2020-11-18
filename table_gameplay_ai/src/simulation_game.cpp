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
#include <visualization_msgs/Marker.h>


#include "ball_prediction.hpp"
#include "table_motor_control/location_conversion.hpp"

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulation_game");

  ros::NodeHandle np("~");
  ros::NodeHandle n;

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1);

  tracking::BallTracker foosball(0);

  ros::Rate r(80);

  foosball.testExtrinsicResults();

  sensor_msgs::JointState joint_msg;

  std::vector<std::string> joint_names = {"white_attack_rot_joint", "white_attack_lin_joint", "white_goalie_rot_joint", "white_goalie_lin_joint", "grey_attack_rot_joint", "grey_attack_lin_joint", "grey_goalie_rot_joint", "grey_goalie_lin_joint"};
  joint_msg.name = joint_names;

  std::pair<double,double> xrange = foosball.getXRange();
  std::pair<double,double> yrange = foosball.getYRange();

  visualization_msgs::Marker ball_marker;

  ball_marker.header.frame_id = "field";

  while(ros::ok())
  {
    std::vector<double> joint_vals(joint_names.size(), 0);

    // Get the position of the ball
    cv::Point3d pos = foosball.getWorldPosition();

    ROS_INFO_STREAM(pos);

    double lin_pos = location_conversion::getLinearPosition(pos.y, yrange, std::make_pair<double,double>(-0.045, 0.045));
    double def_rot_pos = location_conversion::getAngularPosition(pos.x, 0.015, 0.067);
    double fwd_rot_pos = location_conversion::getAngularPosition(pos.x, 0.015, 0.257);

    joint_vals.at(0) = fwd_rot_pos;
    joint_vals.at(1) = lin_pos;
    joint_vals.at(2) = def_rot_pos;
    joint_vals.at(3) = lin_pos;

    joint_msg.header.stamp = ros::Time::now();
    joint_msg.position = joint_vals;

    joint_pub.publish(joint_msg);

    ball_marker.header.stamp = ros::Time::now();
    ball_marker.id = 0;
    ball_marker.type = visualization_msgs::Marker::SPHERE;
    ball_marker.action = visualization_msgs::Marker::ADD;
    ball_marker.pose.position.x = pos.x;
    ball_marker.pose.position.y = pos.y;
    ball_marker.pose.position.z = 0.01375;
    ball_marker.pose.orientation.x = 0.0;
    ball_marker.pose.orientation.y = 0.0;
    ball_marker.pose.orientation.z = 0.0;
    ball_marker.pose.orientation.w = 1.0;

    ball_marker.scale.x = 0.0275;
    ball_marker.scale.y = 0.0275;
    ball_marker.scale.z = 0.0275;

    ball_marker.color.a = 1.0;
    ball_marker.color.r = 0.1;
    ball_marker.color.g = 0.1;
    ball_marker.color.b = 0.1;

    vis_pub.publish(ball_marker);

    ros::spinOnce();

    r.sleep();
  }

  return 0;
}

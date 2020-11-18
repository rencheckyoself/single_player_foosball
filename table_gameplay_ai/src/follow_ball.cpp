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
#include "table_motor_control/tic_server.hpp"
#include "table_motor_control/location_conversion.hpp"

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_ball");

  ros::NodeHandle np("~");
  ros::NodeHandle n;

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1);


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

  tic_server::TicCtrlr def_rot(def_rot_sn, def_rot_nickname);
  tic_server::TicCtrlr def_lin(def_lin_sn, def_lin_nickname);

  tic_server::TicCtrlr fwd_rot(fwd_rot_sn, fwd_rot_nickname);
  tic_server::TicCtrlr fwd_lin(fwd_lin_sn, fwd_lin_nickname);

  def_rot.resume();
  def_lin.resume();

  fwd_rot.resume();
  fwd_lin.resume();

  def_rot.reset_global_position();
  def_lin.reset_global_position();

  fwd_rot.reset_global_position();
  fwd_lin.reset_global_position();

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

    // ROS_INFO_STREAM(pos);

    double lin_pos = location_conversion::getLinearPosition(pos.y, yrange, std::make_pair<double,double>(-0.045, 0.045));
    double def_rot_pos = location_conversion::getAngularPosition(pos.x, 0.025, 0.067);
    double fwd_rot_pos = location_conversion::getAngularPosition(pos.x, 0.025, 0.257);

    int lin_stepper = std::floor(location_conversion::map_ranges(lin_pos, -0.045, 0.045, 0, 240));

    int def_rot_stepper = std::floor(location_conversion::map_ranges(def_rot_pos, -3.14, 3.14, -100, 100));
    int fwd_rot_stepper = std::floor(location_conversion::map_ranges(fwd_rot_pos, -3.14, 3.14, -100, 100));

    if(std::abs(lin_stepper - def_lin.get_current_pos()) > 10)
    {
      def_lin.set_position(lin_stepper);
      fwd_lin.set_position(lin_stepper);
    }

    def_rot.set_position(def_rot_stepper);
    fwd_rot.set_position(fwd_rot_stepper);

    ROS_INFO_STREAM("Stepper pos: " << def_rot.get_current_pos());

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

  fwd_rot.deenergize();
  fwd_lin.deenergize();

  def_rot.deenergize();
  def_lin.deenergize();

  return 0;
}

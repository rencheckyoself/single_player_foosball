/// \file
/// \brief Node to subsribe to commands and send them to the Tic's. Also offers services for manual control.
/// PARAMETERS:
/// PUBLISHES:
///
/// SUBSCRIBES:
///
/// SERIVCES:

#include <utility>
#include <memory>

#include <ros/ros.h>

#include <XmlRpcValue.h>

#include <sensor_msgs/JointState.h>
#include <visualization_msgs/Marker.h>

#include "table_vision_sensing/RodState.h"

#include "ball_prediction.hpp"
#include "table_ai.hpp"

static tracking::RodState fwd_rod_state, def_rod_state;

void forwardRodCB(table_vision_sensing::RodState msg)
{
  fwd_rod_state.rod_is_up = msg.rod_is_up;
  fwd_rod_state.players_are_back = msg.players_are_back;
}

void defenseRodCB(table_vision_sensing::RodState msg)
{
  def_rod_state.rod_is_up = msg.rod_is_up;
  def_rod_state.players_are_back = msg.players_are_back;
}

/// \brief main function to create the real_waypoints node
int main(int argc, char** argv)
{
  ros::init(argc, argv, "follow_ball");

  ros::NodeHandle np("~");
  ros::NodeHandle n;

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>( "visualization_marker", 1);

  ros::Subscriber def_rod_sub = n.subscribe<table_vision_sensing::RodState>("/Def_RodState", 1, defenseRodCB);
  ros::Subscriber fwd_rod_sub = n.subscribe<table_vision_sensing::RodState>("/Fwd_RodState", 1, forwardRodCB);

  XmlRpc::XmlRpcValue world_coordinates;
  XmlRpc::XmlRpcValue image_coordinates;
  double error_threshold;

  table::ControllerInfo tic_info;
  table::CalibrationVals config;

  int mode;

  np.getParam("mode", mode);

  // Read in homography configuration
  np.getParam("world_coordinates", world_coordinates);
  np.getParam("image_coordinates", image_coordinates);
  np.getParam("error_threshold", error_threshold);

  // Read in tic controller configurations
  np.getParam("fwd_rot/serial_number", tic_info.fwd_rot_sn);
  np.getParam("fwd_rot/nickname", tic_info.fwd_rot_nickname);

  np.getParam("fwd_lin/serial_number", tic_info.fwd_lin_sn);
  np.getParam("fwd_lin/nickname", tic_info.fwd_lin_nickname);

  np.getParam("def_rot/serial_number", tic_info.def_rot_sn);
  np.getParam("def_rot/nickname", tic_info.def_rot_nickname);

  np.getParam("def_lin/serial_number", tic_info.def_lin_sn);
  np.getParam("def_lin/nickname", tic_info.def_lin_nickname);

  // Read in configuration parameters
  np.getParam("lower_joint_limit", config.lin_joint_limits.first);
  np.getParam("upper_joint_limit", config.lin_joint_limits.second);

  np.getParam("lower_lin_steps", config.lin_step_limits.first);
  np.getParam("upper_lin_steps", config.lin_step_limits.second);

  np.getParam("linear_hysteresis", config.linear_hysteresis);

  np.getParam("kick_trigger_threshold", config.kick_trigger_threshold);

  np.getParam("def_rod_xpos", config.def_rod_xpos);
  np.getParam("fwd_rod_xpos", config.fwd_rod_xpos);

  np.getParam("velocity_modifier", config.velocity_modifier);

  np.getParam("def_reset_offset", config.def_reset_offset);
  np.getParam("fwd_reset_offset", config.fwd_reset_offset);
  np.getParam("full_rotation_offset", config.full_rotation_offset);

  ROS_INFO_STREAM("TICCMD: Forward Rotation Name: " << tic_info.fwd_rot_nickname);
  ROS_INFO_STREAM("TICCMD: Forward Rotation ID: " << tic_info.fwd_rot_sn);

  ROS_INFO_STREAM("TICCMD: Forward Linear Name: " << tic_info.fwd_lin_nickname);
  ROS_INFO_STREAM("TICCMD: Forward Linear ID: " << tic_info.fwd_lin_sn);

  ROS_INFO_STREAM("TICCMD: Defense Rotation Name: " << tic_info.def_rot_nickname);
  ROS_INFO_STREAM("TICCMD: Defense Rotation ID: " << tic_info.def_rot_sn);

  ROS_INFO_STREAM("TICCMD: Defense Linear Name: " << tic_info.def_lin_nickname);
  ROS_INFO_STREAM("TICCMD: Defense Linear ID: " << tic_info.def_lin_sn);

  std::vector<std::vector<double>> wc = tracking::parse_points_data(world_coordinates);
  std::vector<std::vector<double>> ic = tracking::parse_points_data(image_coordinates);

  tracking::BallTracker foosball(0, wc, ic, error_threshold);
  foosball.testExtrinsicResults();

  config.xrange = foosball.getXRange();
  config.yrange = foosball.getYRange();

  std::unique_ptr<table::RealFoosballTable> robot_player;

  switch(mode)
  {
    case 0:
    {
      robot_player = std::unique_ptr<table::RealFoosballTable>(new table::RealFoosballTable(tic_info, config));
      break;
    }
    case 1:
    {
      robot_player = std::unique_ptr<table::RealFoosballTable>(new table::DeathSpinTable(tic_info, config));
      break;
    }
    case 2:
    {
      robot_player = std::unique_ptr<table::RealFoosballTable>(new table::FeedbackTable(tic_info, config));
      break;
    }
    default:
    {
      ROS_ERROR_STREAM("Invalid Mode. Must use 0 (open loop mode), 1 (spin mode), or 2 (rotational feedback mode).");
      ros::shutdown();
    }
  }

  ros::Rate r(100);

  sensor_msgs::JointState joint_msg;
  std::vector<std::string> joint_names = {"white_attack_rot_joint", "white_attack_lin_joint", "white_goalie_rot_joint", "white_goalie_lin_joint", "grey_attack_rot_joint", "grey_attack_lin_joint", "grey_goalie_rot_joint", "grey_goalie_lin_joint"};
  joint_msg.name = joint_names;

  visualization_msgs::Marker ball_marker;
  ball_marker.header.frame_id = "field";

  while(ros::ok())
  {
    // Get the position of the ball
    cv::Point3d pos = foosball.getWorldPosition();

    robot_player->moveTable(pos, fwd_rod_state, def_rod_state);

    std::vector<double> joint_vals = robot_player->getCurrentJointStates();

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

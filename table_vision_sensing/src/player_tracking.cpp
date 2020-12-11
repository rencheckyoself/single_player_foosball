/// \file
/// \brief Node to detect the rough angle of the automated rods
/// PARAMETERS:
///     table_vision_sensing/config/player_angle_detection.yaml
/// PUBLISHES:
///   /Def_RodState (table_vision_sensing/RosState) Rod state information for the defensive rod
///   /Fwd_RodState (table_vision_sensing/RosState) Rod state information for the offensive rod
/// SUBSCRIBES:
///   /camera/image_rect_color the image from the overhead camera

#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>

#include "player_angle_detection.hpp"
#include "table_vision_sensing/RodState.h"

/// \brief Class to use the ROS Image transport and work with an image.
class ImageConverter
{
  ros::NodeHandle nh_; ///< node handle
  image_transport::ImageTransport it_; ///< image transport
  image_transport::Subscriber image_sub_; ///< image subscriber

  ros::Publisher def_state_pub, fwd_state_pub; ///< publishers for the rod states

  tracking::PlayerDetector def_rod, fwd_rod; ///< classes to run the angle detection

  tracking::PlayerDetectionParams fwd_rod_params, def_rod_params; ///< struct to hold the detection configuration params

public:

  /// \brief default constructor. Initialze the image transport and get the parameters
  ImageConverter() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect", 1, &ImageConverter::imageCb, this);

    def_state_pub = nh_.advertise<table_vision_sensing::RodState>("/Def_RodState", 1);
    fwd_state_pub = nh_.advertise<table_vision_sensing::RodState>("/Fwd_RodState", 1);

    ros::NodeHandle np("~");

    // Read in angle detection params
    np.getParam("fwd/roi_x", fwd_rod_params.roi_x);
    np.getParam("fwd/roi_y", fwd_rod_params.roi_y);
    np.getParam("fwd/roi_width", fwd_rod_params.roi_width);
    np.getParam("fwd/roi_height", fwd_rod_params.roi_height);
    np.getParam("fwd/area_limit", fwd_rod_params.area_limit);
    np.getParam("fwd/end_kick_width", fwd_rod_params.end_kick_width);
    np.getParam("fwd/end_kick_x", fwd_rod_params.end_kick_x);

    np.getParam("def/roi_x", def_rod_params.roi_x);
    np.getParam("def/roi_y", def_rod_params.roi_y);
    np.getParam("def/roi_width", def_rod_params.roi_width);
    np.getParam("def/roi_height", def_rod_params.roi_height);
    np.getParam("def/area_limit", def_rod_params.area_limit);
    np.getParam("def/end_kick_width", def_rod_params.end_kick_width);
    np.getParam("def/end_kick_x", def_rod_params.end_kick_x);

    // initialize detection
    fwd_rod = tracking::PlayerDetector(fwd_rod_params);
    def_rod = tracking::PlayerDetector(def_rod_params);
  }

  /// \brief callback for the image subscriber
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    // Attempt to copy the ROS image into opencv image
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Detect the rod states
    tracking::RodState def_state = def_rod.detectRodAngle(cv_ptr);
    tracking::RodState fwd_state = fwd_rod.detectRodAngle(cv_ptr);

    // Construct the rod state messages
    table_vision_sensing::RodState def_output, fwd_output;

    def_output.rod_is_up = def_state.rod_is_up;
    def_output.players_are_back = def_state.players_are_back;
    def_output.min_x_value = def_state.boundRect.x;

    // position the player bounding window relative to the full image
    def_output.bounding_rect_img.x_offset = def_state.boundRect.x + def_rod_params.roi_x;
    def_output.bounding_rect_img.y_offset = def_state.boundRect.y + def_rod_params.roi_y;
    def_output.bounding_rect_img.width = def_state.boundRect.width;
    def_output.bounding_rect_img.height = def_state.boundRect.height;

    def_output.window_roi.x_offset = def_rod_params.roi_x;
    def_output.window_roi.y_offset = def_rod_params.roi_y;
    def_output.window_roi.width = def_rod_params.roi_width;
    def_output.window_roi.height = def_rod_params.roi_height;

    fwd_output.rod_is_up = fwd_state.rod_is_up;
    fwd_output.players_are_back = fwd_state.players_are_back;
    fwd_output.min_x_value = fwd_state.boundRect.x;

    // position the player bounding window relative to the full image
    fwd_output.bounding_rect_img.x_offset = fwd_state.boundRect.x + fwd_rod_params.roi_x;
    fwd_output.bounding_rect_img.y_offset = fwd_state.boundRect.y + fwd_rod_params.roi_y;
    fwd_output.bounding_rect_img.width = fwd_state.boundRect.width;
    fwd_output.bounding_rect_img.height = fwd_state.boundRect.height;

    fwd_output.window_roi.x_offset = fwd_rod_params.roi_x;
    fwd_output.window_roi.y_offset = fwd_rod_params.roi_y;
    fwd_output.window_roi.width = fwd_rod_params.roi_width;
    fwd_output.window_roi.height = fwd_rod_params.roi_height;

    def_state_pub.publish(def_output);
    fwd_state_pub.publish(fwd_output);
  }
};

/// \brief main function to create the real_waypoints node
/// \param argc argument count
/// \param arguments
/// \returns success
int main(int argc, char** argv)
{
  ros::init(argc, argv, "background_removal");
  ImageConverter ic;
  ros::spin();
  return 0;
}

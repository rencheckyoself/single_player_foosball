/// \file
/// \brief Use OpenCV to display the image output
/// PARAMETERS:
///   point_radius: the radius of the dot drawn for the balls location
///   show_ball_pos: set to false to hide the ball location dot
///   show_player_angle: set to false to hide the player detection rectangles
/// PUBLISHES:
/// SUBSCRIBES:
///   /camera/image_rect_color the image from the overhead camera
///   /BallPosition (geometry_msgs/Point) The image coordinates of the ball
///   /Def_RodState (table_vision_sensing/RosState) Rod state information for the defensive rod
///   /Fwd_RodState (table_vision_sensing/RosState) Rod state information for the offensive rod

#include <vector>
#include <string>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "table_vision_sensing/RodState.h"

/// \brief Class to use the ROS Image transport and work with an image.
class ImageConverter
{
  ros::NodeHandle nh_; ///< node handle
  image_transport::ImageTransport it_; ///< image transport
  image_transport::Subscriber image_sub_; ///< image subscriber

  ros::Subscriber ball_pos_sub; ///< ball position subscriber
  ros::Subscriber def_rect_sub; ///< defensive rod state subscriber
  ros::Subscriber fwd_rect_sub; ///< offensive rod state subscriber

  ros::Time last_frame_time; ///< the time the last frame was recieved
  ros::Time current_frame_time; ///< the time the current frame was recieved

  int point_radius = 5; ///< radius of the dot for the balls location

  cv::Point ball_loc; ///< the most recent balls image coordinates

  cv::Rect def_rect, fwd_rect; ///< bounding rectangles for the detected players
  cv::Rect def_window, fwd_window; ///< bounding rectangles for the detection windows

  bool def_up = false; ///< if the defensive rod is up
  bool def_back = false; ///< if the defensive rod is back
  bool fwd_up = false; ///< if the offensive rod is up
  bool fwd_back = false; ///< if the offensive rod is back

  bool show_ball_pos = true; ///< flag to draw the balls position on the image
  bool show_player_angle = true; ///< flag to draw the player detection rectangles on the image

public:

  /// \brief default constructor. Initialze the image transport and get the parameters
  ImageConverter() : it_(nh_)
  {
    ros::NodeHandle np("~");

    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);

    ball_pos_sub = nh_.subscribe("BallPosition", 1, &ImageConverter::ballPosCB, this);
    def_rect_sub = nh_.subscribe("Def_RodState", 1, &ImageConverter::defRodCB, this);
    fwd_rect_sub = nh_.subscribe("Fwd_RodState", 1, &ImageConverter::fwdRodCB, this);

    np.getParam("point_radius", point_radius);
    np.getParam("show_ball_pos", show_ball_pos);
    np.getParam("show_player_angle", show_player_angle);

    current_frame_time = ros::Time::now();
    last_frame_time = ros::Time::now();

    def_rect = cv::Rect(0,0,0,0);
    fwd_rect = cv::Rect(0,0,0,0);
  }

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

    // Calculate the FPS
    int fps = 1.0 / (msg->header.stamp - last_frame_time).toSec();
    last_frame_time = msg->header.stamp;

    // Draw the FPS Counter
    cv::putText(cv_ptr->image, std::to_string(fps), cv::Point(0,30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1);

    // Draw the ball position
    if(show_ball_pos) cv::circle(cv_ptr->image, ball_loc, point_radius, cv::Scalar(0,0,255), -1);

    // Draw the Rod Bounding Boxes
    if(show_player_angle)
    {
      cv::Scalar color;

      if(def_up && def_back) color = cv::Scalar(0,0,255);
      else if(def_up) color = cv::Scalar(0,255,0);
      else color = cv::Scalar(255,0,0);

      cv::rectangle(cv_ptr->image, def_rect, color, 2);

      if(fwd_up && fwd_back) color = cv::Scalar(0,0,255);
      else if(fwd_up) color = cv::Scalar(0,255,0);
      else color = cv::Scalar(255,0,0);

      cv::rectangle(cv_ptr->image, fwd_rect, color, 2);

      cv::rectangle(cv_ptr->image, def_window, cv::Scalar(255,255,255), 2);
      cv::rectangle(cv_ptr->image, fwd_window, cv::Scalar(255,255,255), 2);
    }

    // Update GUI Window
    cv::imshow("Overhead Camera", cv_ptr->image);

    cv::waitKey(1);
  }

  /// \brief ball position suscriber
  /// \param msg ball image coordinates
  void ballPosCB(const geometry_msgs::Point& msg)
  {
    ball_loc.x = msg.x;
    ball_loc.y = msg.y;
  }

  /// \brief defensive rod state subscriber callback
  /// \msg defensive rod state
  void defRodCB(const table_vision_sensing::RodState msg)
  {
    def_up = msg.rod_is_up;
    def_back = msg.players_are_back;

    def_rect = cv::Rect(msg.bounding_rect_img.x_offset, msg.bounding_rect_img.y_offset, msg.bounding_rect_img.width, msg.bounding_rect_img.height);
    def_window = cv::Rect(msg.window_roi.x_offset, msg.window_roi.y_offset, msg.window_roi.width, msg.window_roi.height);
  }

  /// \brief offensive rod state subscriber callback
  /// \msg offensive rod state
  void fwdRodCB(const table_vision_sensing::RodState msg)
  {
    fwd_up = msg.rod_is_up;
    fwd_back = msg.players_are_back;

    fwd_rect = cv::Rect(msg.bounding_rect_img.x_offset, msg.bounding_rect_img.y_offset, msg.bounding_rect_img.width, msg.bounding_rect_img.height);
    fwd_window = cv::Rect(msg.window_roi.x_offset, msg.window_roi.y_offset, msg.window_roi.width, msg.window_roi.height);
  }


};

/// \brief main function to create the real_waypoints node
/// \param argc argument count
/// \param arguments
/// \returns success
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cascade_classifier");
  ImageConverter ic;
  ros::spin();
  return 0;
}

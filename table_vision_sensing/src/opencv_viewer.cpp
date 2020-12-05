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

class ImageConverter
{
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Subscriber ball_pos_sub;
  ros::Subscriber def_rect_sub;
  ros::Subscriber fwd_rect_sub;

  ros::Time last_frame_time;
  ros::Time current_frame_time;

  int point_radius = 5;

  cv::Point ball_loc;

  cv::Rect def_rect, fwd_rect;
  cv::Rect def_window, fwd_window;

  bool def_state = false;
  bool fwd_state = false;

  bool show_ball_pos = true;
  bool show_player_angle = true;

  std::string video_name = "default";

public:
  ImageConverter() : it_(nh_)
  {
    ros::NodeHandle np("~");
    ros::NodeHandle n;

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);

    ball_pos_sub = n.subscribe("BallPosition", 1, &ImageConverter::ballPosCB, this);
    def_rect_sub = n.subscribe("Def_RodState", 1, &ImageConverter::defRodCB, this);
    fwd_rect_sub = n.subscribe("Fwd_RodState", 1, &ImageConverter::fwdRodCB, this);

    np.getParam("point_radius", point_radius);
    np.getParam("show_ball_pos", show_ball_pos);
    np.getParam("show_player_angle", show_player_angle);
    np.getParam("video_name", video_name);


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

      if(def_state) color = cv::Scalar(0,255,0);
      else color = cv::Scalar(255,0,0);

      cv::rectangle(cv_ptr->image, def_rect, color, 2);

      if(fwd_state) color = cv::Scalar(0,255,0);
      else color = cv::Scalar(255,0,0);

      cv::rectangle(cv_ptr->image, fwd_rect, color, 2);

      cv::rectangle(cv_ptr->image, def_window, cv::Scalar(0,0,255), 2);
      cv::rectangle(cv_ptr->image, fwd_window, cv::Scalar(0,0,255), 2);
    }

    // Update GUI Window
    cv::imshow("Overhead Camera", cv_ptr->image);

    cv::waitKey(1);
  }

  void ballPosCB(const geometry_msgs::Point& msg)
  {
    ball_loc.x = msg.x;
    ball_loc.y = msg.y;
  }

  void defRodCB(const table_vision_sensing::RodState msg)
  {
    def_state = msg.rod_is_up;

    def_rect = cv::Rect(msg.bounding_rect_img.x_offset, msg.bounding_rect_img.y_offset, msg.bounding_rect_img.width, msg.bounding_rect_img.height);
    def_window = cv::Rect(msg.window_roi.x_offset, msg.window_roi.y_offset, msg.window_roi.width, msg.window_roi.height);
  }

  void fwdRodCB(const table_vision_sensing::RodState msg)
  {
    fwd_state = msg.rod_is_up;

    fwd_rect = cv::Rect(msg.bounding_rect_img.x_offset, msg.bounding_rect_img.y_offset, msg.bounding_rect_img.width, msg.bounding_rect_img.height);
    fwd_window = cv::Rect(msg.window_roi.x_offset, msg.window_roi.y_offset, msg.window_roi.width, msg.window_roi.height);
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cascade_classifier");
  ImageConverter ic;
  ros::spin();
  return 0;
}

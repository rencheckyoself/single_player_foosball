#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>


#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

class ImageConverter
{
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Subscriber ball_pos_sub;
  ros::Subscriber camera_info_sub;

  ros::Time last_frame_time;
  ros::Time current_frame_time;

  int point_radius = 5;

  image_geometry::PinholeCameraModel cam_model_;

  cv::Point ball_loc;

public:
  ImageConverter() : it_(nh_)
  {
    ros::NodeHandle np("~");
    ros::NodeHandle n;

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    camera_info_sub = n.subscribe("/camera/camera_info", 1, &ImageConverter::cameraInfoCb, this);

    ball_pos_sub = n.subscribe("BallPosition", 1, &ImageConverter::ballPosCB, this);

    np.getParam("point_radius", point_radius);

    current_frame_time = ros::Time::now();
    last_frame_time = ros::Time::now();
  }

  ~ImageConverter() {}

  void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    // ROS_INFO_STREAM("Set Camera Model");
    cam_model_.fromCameraInfo(info_msg);
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

    cv::circle(cv_ptr->image, ball_loc, 5, cv::Scalar(0,0,255), -1);
    ROS_INFO_STREAM("Ball Coordinates: " << ball_loc.x << ", " << ball_loc.y);

    cv::Point origin = cam_model_.project3dToPixel(cv::Point3d(0, 0, 100));

    cv::circle(cv_ptr->image, origin, 2, cv::Scalar(0,255,0), -1);

    // cv::Point3d ball_pos = cam_model_.projectPixelTo3dRay(ball_loc);
    // ROS_INFO_STREAM("Ball Coordinates: " << ball_pos.x << ", " << ball_pos.y << ", " << ball_pos.z);

    cv::putText(cv_ptr->image, std::to_string(fps), cv::Point(0,30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 1);

    // Update GUI Window
    cv::imshow("Overhead Camera", cv_ptr->image);

    cv::waitKey(1);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());

  }

  void ballPosCB(const geometry_msgs::Point& msg)
  {
    ball_loc.x = msg.x;
    ball_loc.y = msg.y;
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cascade_classifier");
  ImageConverter ic;
  ros::spin();
  return 0;
}

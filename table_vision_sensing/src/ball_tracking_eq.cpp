/// \file
/// \brief Node pull the image from the overhead camera and identify the ball's location
/// PARAMETERS:
/// PUBLISHES:
///
/// SUBSCRIBES:
///
/// SERIVCES:

#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int blur_val_ = 5;

public:
  ImageConverter() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    ros::NodeHandle n("~");

    n.param("blur_val", blur_val_, 5);

    ROS_INFO_STREAM("Using blur of: " << blur_val_);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    cv::Mat grey;
    cv::cvtColor(cv_ptr->image, grey, cv::COLOR_BGR2GRAY);

    cv::equalizeHist(grey, grey);

    cv::medianBlur(grey, grey, blur_val_);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(grey, circles, cv::HOUGH_GRADIENT, 1, 5, 100, 22, 10, 18);

    for( size_t i = 0; i < circles.size(); i++ )
    {
      cv::Vec3i c = circles[i];
      cv::Point center = cv::Point(c[0], c[1]);
      // circle center
      cv::circle(cv_ptr->image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
      cv::circle(grey, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
      // circle outline
      int radius = c[2];
      cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, cv::LINE_AA);
      cv::circle(grey, center, radius, cv::Scalar(255,0,255), 3, cv::LINE_AA);
    }

    // Update GUI Window
    ROS_INFO_STREAM("EQ Found " << circles.size() << " circles.");
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::imshow("Equalized", grey);


    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_tracking");
  ImageConverter ic;
  ros::spin();
  return 0;
}

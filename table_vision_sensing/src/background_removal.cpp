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
#include <opencv2/video/video.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Ptr<cv::BackgroundSubtractor> pBackSub;
  cv::Mat fgMask;

public:
  ImageConverter() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/fg_mask", 1);

    // intialize background subtraction
    pBackSub = cv::createBackgroundSubtractorKNN(350, 10000.0, false);

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
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

    // cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2HSV);

    pBackSub->apply(cv_ptr->image, fgMask);

    // Update GUI Window
    cv::imshow("Rectified", cv_ptr->image);

    cv::Mat masked_image;
    cv::cvtColor(fgMask, masked_image, cv::COLOR_GRAY2BGR);
    cv::bitwise_and(cv_ptr->image, masked_image, masked_image);
    cv::imshow("Masked Image", masked_image);

    cv::waitKey(3);

    // Output modified video stream

    sensor_msgs::ImagePtr mask_out = cv_bridge::CvImage(std_msgs::Header(), "mono8", fgMask).toImageMsg();

    image_pub_.publish(mask_out);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "background_removal");
  ImageConverter ic;
  ros::spin();
  return 0;
}

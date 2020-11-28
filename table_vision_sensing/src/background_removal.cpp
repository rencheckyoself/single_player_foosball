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

  cv::Scalar blue = cv::Scalar(256, 0, 0);
  cv::Scalar green = cv::Scalar(0, 256, 0);
  cv::Scalar red = cv::Scalar(0, 0, 256);

public:
  ImageConverter() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/fg_mask", 1);

    // intialize background subtraction
    pBackSub = cv::createBackgroundSubtractorKNN(350, 10000.0, false);
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

    cv::Rect roi(50,30,150,180);

    cv::Mat cropped_img = cv_ptr->image(roi);

    pBackSub->apply(cropped_img, fgMask);

    cv::Mat detection_img;

    cv::Mat d_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));
    cv::dilate(fgMask, detection_img, d_element);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(detection_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Point2f> centers( contours.size() );
    std::vector<float> radius( contours.size() );

    for( size_t i = 0; i < contours.size(); i++ )
    {
        boundRect.at(i) = cv::boundingRect(contours[i]);
        cv::minEnclosingCircle(contours[i], centers[i], radius[i]);

        cv::drawContours(detection_img, contours, (int)i, red, 2, cv::LINE_8, hierarchy, 0);
        cv::rectangle(cropped_img, boundRect.at(i), green, 2);
        cv::circle(cropped_img, centers[i], 2, blue, -1);
    }

    // Update GUI Window
    cv::imshow("Rectified", cv_ptr->image);
    cv::imshow("Cropped", cropped_img);

    cv::Mat masked_image;
    // cv::cvtColor(fgMask, masked_image, cv::COLOR_GRAY2BGR);
    // cv::bitwise_and(cv_ptr->image, masked_image, masked_image);
    cv::imshow("Mask", fgMask);

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

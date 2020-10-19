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
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

static const float PI = 3.14195;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Subscriber camera_info_sub;
  int blur_val_ = 5;

  image_geometry::PinholeCameraModel cam_model_;

public:
  ImageConverter() : it_(nh_)
  {
    ros::NodeHandle np("~");
    ros::NodeHandle n;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    camera_info_sub = n.subscribe("/camera/camera_info", 1, &ImageConverter::cameraInfoCb, this);

    np.param("blur_val", blur_val_, 5);

    ROS_INFO_STREAM("Using blur of: " << blur_val_);

    // cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    // cv::destroyWindow(OPENCV_WINDOW);
  }

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

    cv::Mat grey;
    cv::cvtColor(cv_ptr->image, grey, cv::COLOR_BGR2GRAY);

    // blur(grey, grey, cv::Size(9,9));
    // cv::Canny(grey, grey, 0, 255, 3);

    cv::Mat e_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
    cv::Mat d_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7,7));

    cv::dilate(grey, grey, d_element);
    // cv::erode(grey, grey, e_element);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours(grey, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<std::vector<cv::Point> > contours_poly( contours.size() );
    std::vector<cv::Rect> boundRect( contours.size() );
    std::vector<cv::Point2f> centers( contours.size() );
    std::vector<float> radius( contours.size() );

    std::vector<cv::RotatedRect> minEllipse( contours.size() );

    std::vector<std::vector<cv::Point>>hull( contours.size() );

    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::approxPolyDP(contours[i], contours_poly[i], 3, true);
        cv::minEnclosingCircle(contours[i], centers[i], radius[i]);
        if(contours.at(i).size() > 4) minEllipse.at(i) = cv::fitEllipse(contours.at(i));
        cv::convexHull(contours.at(i), hull.at(i));
    }

    for( size_t i = 0; i< contours.size(); i++ )
    {
        cv::Scalar blue = cv::Scalar(256, 0, 0);
        cv::Scalar green = cv::Scalar(0, 256, 0);
        cv::Scalar red = cv::Scalar(0, 0, 256);

        cv::drawContours(cv_ptr->image, contours, (int)i, blue, 2, cv::LINE_8, hierarchy, 0);

        double majora = std::max(minEllipse.at(i).size.height, minEllipse.at(i).size.width);
        double minora = std::min(minEllipse.at(i).size.height, minEllipse.at(i).size.width);

        // if(radius.at(i) <= 25 && radius.at(i) >= 18 && minora/majora > 0.8)
        // {
        //   cv::ellipse(cv_ptr->image, minEllipse.at(i), red);
        //   cv::circle(cv_ptr->image, centers[i], radius[i], green, 2);
        // }

        if(radius.at(i) <= 25 && radius.at(i) >= 18)
        {
          double area = std::abs(cv::contourArea(contours.at(i)) - cv::contourArea(hull.at(i)));
          if( area < 100)
          {
            ROS_INFO_STREAM("Area: " << area);
            cv::drawContours(cv_ptr->image, hull, (int)i, red, 2);
            cv::circle(cv_ptr->image, centers[i], radius[i], green, 2);
          }
        }
    }

    cv::Point3d ball_pos;

    if(contours.size() == 1)
    {
      ball_pos = cam_model_.projectPixelTo3dRay(centers.at(0));

      ROS_INFO_STREAM("Ball Coordinates: " << ball_pos.x << ", " << ball_pos.y << ", " << ball_pos.z);
    }

    // std::vector<cv::Vec3f> circles;
    // cv::HoughCircles(grey, circles, cv::HOUGH_GRADIENT, 1, 1, 1, 100, 0, 0);
    //
    // for( size_t i = 0; i < circles.size(); i++ )
    // {
    //   cv::Vec3i c = circles[i];
    //   cv::Point center = cv::Point(c[0], c[1]);
    //   // circle center
    //   cv::circle(cv_ptr->image, center, 1, cv::Scalar(0,100,100), 3, cv::LINE_AA);
    //   // circle outline
    //   int radius = c[2];
    //   cv::circle(cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, cv::LINE_AA);
    // }
    // ROS_INFO_STREAM("N: Found " << circles.size() << " circles.");

    // Update GUI Window
    cv::imshow("Dilated Mask", grey);
    cv::imshow("Mask", cv_ptr->image);

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

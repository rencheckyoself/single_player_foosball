#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/image_encodings.h>

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
  ros::Subscriber camera_info_sub;

  image_geometry::PinholeCameraModel cam_model_;

  cv::CascadeClassifier ball_finder;

  ros::Time last_frame;

public:
  ImageConverter() : it_(nh_)
  {
    ros::NodeHandle np("~");
    ros::NodeHandle n;

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // Load the trained model
    if( !ball_finder.load("/home/michaelrencheck/FinalProject/src/table_vision_sensing/cascade_data/cascade.xml"))
    {
      ROS_ERROR_STREAM("Casade not loaded properly");
      ros::shutdown();
    }

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

    std::vector<cv::Rect> detected_balls;
    std::vector<double> weights;
    std::vector<int> levels;
    cv::Mat grey;

    cv::cvtColor(cv_ptr->image, grey, cv::COLOR_BGR2GRAY);

    ball_finder.detectMultiScale(cv_ptr->image, detected_balls, levels, weights, 1.1, 3, 0, cv::Size(), cv::Size(50,50), true);

    // for(auto rect : detected_balls)
    for(unsigned int i = 0; i < detected_balls.size(); i++)
    {
      cv::Rect rect = detected_balls.at(i);
      cv::Point center(rect.x + rect.width/2, rect.y + rect.height/2);
      cv::rectangle(cv_ptr->image, rect, cv::Scalar(0, 0, 255), 3);
      cv::putText(cv_ptr->image, std::to_string(weights.at(i)), center, cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 2);
    }

    if(weights.size() > 0)
    {
      int max_index = std::distance( weights.begin(), std::max_element(weights.begin(), weights.end()));
      cv::rectangle(cv_ptr->image, detected_balls.at(max_index), cv::Scalar(255,0,0), 3);
    }

    int fps = 1.0 / (msg->header.stamp - last_frame).toSec();
    last_frame = msg->header.stamp;

    // ROS_INFO_STREAM("FPS: " << fps);

    cv::putText(cv_ptr->image, std::to_string(fps), cv::Point(0,30), cv::FONT_HERSHEY_DUPLEX, 1, cv::Scalar(0, 255, 0), 1);

    // Update GUI Window
    cv::imshow("Mask", cv_ptr->image);

    cv::waitKey(1);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cascade_classifier");
  ImageConverter ic;
  ros::spin();
  return 0;
}

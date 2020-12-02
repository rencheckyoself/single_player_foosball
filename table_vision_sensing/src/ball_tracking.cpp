#include <vector>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

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

  ros::Publisher ball_pos;

  cv::CascadeClassifier ball_finder;

  ros::Time last_frame;

public:
  ImageConverter() : it_(nh_)
  {
    ros::NodeHandle np("~");
    ros::NodeHandle n;

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);

    ball_pos = n.advertise<geometry_msgs::Point>("BallPosition", 1);

    // Load the trained model
    if(!ball_finder.load("/home/michaelrencheck/FinalProject/src/table_vision_sensing/cascade_data/cascade.xml"))
    {
      ROS_ERROR_STREAM("Cascade not loaded properly");
      ros::shutdown();
    }
  }

  ~ImageConverter() {}

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

    ball_finder.detectMultiScale(cv_ptr->image, detected_balls, levels, weights, 1.1, 3, 0, cv::Size(), cv::Size(50,50), true);

    geometry_msgs::Point ball_loc;

    if(weights.size() > 0)
    {
      int max_index = std::distance( weights.begin(), std::max_element(weights.begin(), weights.end()));
      ball_loc.x = detected_balls.at(max_index).x + detected_balls.at(max_index).width/2;
      ball_loc.y = detected_balls.at(max_index).y + detected_balls.at(max_index).height/2;
    }

    ball_pos.publish(ball_loc);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_tracking");
  ImageConverter ic;
  ros::spin();
  return 0;
}

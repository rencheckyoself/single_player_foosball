/// \file
/// \brief Node to track the ball using OpenCVs cascade classifer.
/// PUBLISHES:
///   /BallPosition (geometry_msgs/Point) The image coordinates of the ball
/// SUBSCRIBES:
///   /camera/image_rect_color the image from the overhead camera

#include <vector>
#include <string>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

/// \brief Class to use the ROS Image transport and work with an image.
class ImageConverter
{
  ros::NodeHandle nh_; ///< node handle
  image_transport::ImageTransport it_; ///< image transport
  image_transport::Subscriber image_sub_; ///< subscriber for the image topic

  ros::Publisher ball_pos; ///< pulisher for the ball's image coordinates

  cv::CascadeClassifier ball_finder; ///< the classier to detect the ball

  ros::Time last_frame; ///< time the previous frame was recieved.

public:

  /// \brief Default constructor. Initialize the classifier and image transport.
  ImageConverter() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);

    ball_pos = nh_.advertise<geometry_msgs::Point>("BallPosition", 1);

    // Get the package location
    std::string package_path = ros::package::getPath("table_vision_sensing");

    // Load the trained model
    if(!ball_finder.load(package_path + "/cascade_data/cascade.xml"))
    {
      ROS_ERROR_STREAM("Cascade not loaded properly");
      ros::shutdown();
    }
  }

  /// \brief Callback for the image subscriber
  /// \param msg the ros image
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

    // detect if there is a ball
    // Modify the two values after 'weights' to change how accurate the classifer, see OpenCV documentation
    // By lowering the 6th argument, the ball will be detected more often, but will also result in more false positives.
    // By lowering the 5th argument, the detection algorithm will be more likely to find the ball, but will run slower.
    ball_finder.detectMultiScale(cv_ptr->image, detected_balls, levels, weights, 1.4, 3, 0, cv::Size(), cv::Size(50,50), true);

    geometry_msgs::Point ball_loc;

    // check if a ball was detected.
    if(weights.size() > 0)
    {
      // find the detected object with the highest weight and assume it is the ball
      int max_index = std::distance( weights.begin(), std::max_element(weights.begin(), weights.end()));
      ball_loc.x = detected_balls.at(max_index).x + detected_balls.at(max_index).width/2;
      ball_loc.y = detected_balls.at(max_index).y + detected_balls.at(max_index).height/2;
    }

    ball_pos.publish(ball_loc);
  }

};

/// \brief main function to create the real_waypoints node
/// \param argc argument count
/// \param arguments
/// \returns success
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_tracking");
  ImageConverter ic;
  ros::spin();
  return 0;
}

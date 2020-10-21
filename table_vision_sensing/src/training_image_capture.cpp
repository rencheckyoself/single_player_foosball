#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
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

  std::string package_path;
  std::string file_path;
  std::string full_directory;

public:
  ImageConverter() : it_(nh_)
  {
    ros::NodeHandle np("~");
    ros::NodeHandle n;

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);

    package_path = ros::package::getPath("table_vision_sensing");
    np.getParam("file_path", file_path);

    full_directory = package_path + file_path;

    ROS_INFO_STREAM("ImCa: Using Directory:" << full_directory);
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

    cv::imshow("Testing", cv_ptr->image);

    char key = static_cast<char>(cv::waitKey(1));

    if(key == 's')
    {
      cv::imwrite(full_directory + "test.jpg", cv_ptr->image);
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_tracking");
  ImageConverter ic;
  ros::spin();
  return 0;
}

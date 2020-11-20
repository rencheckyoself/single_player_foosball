/// \file This is a ROS node to save images from an image topic in order to use for the cascade classifier in opencv.

#include <vector>
#include <fstream>

#include <sys/stat.h>
#include <sys/types.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

/// \brief Class to perform the image capture.
class ImageConverter
{
  ros::NodeHandle nh_; ///< node hanlde for image transport
  image_transport::ImageTransport it_; ///< image transport
  image_transport::Subscriber image_sub_; ///< image subscriber

  std::string image_group; ///< the name of the directory and info_file for a set of images

  std::string package_path; ///< path to the package
  std::string file_path; ///< the subdirectory in the package that contains all of the training data
  std::string full_directory; ///< the full path to the training directory

  std::string image_name; ///< the relative path to the image starting in the training directory

  bool generate_annotation; ///< if true, add an annotation for the negative image set

  std::fstream info_file; ///< file for annotation

  int count; ///< used to uniquely name images. Increments as they are saved.

  // Used for cropping
  bool done_drawing;
  cv::Point start_corner;
  cv::Point end_corner;
  cv::Mat img;

public:
  ImageConverter() : it_(nh_)
  {
    ros::NodeHandle np("~");
    ros::NodeHandle n;

    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_rect_color", 1, &ImageConverter::imageCb, this);

    package_path = ros::package::getPath("table_vision_sensing");
    np.getParam("file_path", file_path);
    np.getParam("image_group", image_group);
    np.getParam("generate_annotation", generate_annotation);

    file_path = "/" + file_path; // "/training_data/"
    full_directory = package_path + file_path; // [package_path]/training_data/
    count = 0;

    // Ensure the image group directory is created
    mkdir((full_directory).c_str(), 0777);
    mkdir((full_directory + image_group).c_str(), 0777);

    ROS_INFO_STREAM(image_group << "Capture: Using Directory:" << full_directory + image_group);

    info_file.open(full_directory + image_group + ".dat", std::ios::in | std::ios::out | std::ios::app | std::ios::binary);

    // get to last character of file
    info_file.seekg(-1,std::ios_base::end);

    if(info_file.peek() == '\n') // check if the last character is a new line
    {
      std::string digits;
      info_file.seekg(-1,std::ios_base::cur);

      bool g_found = false;

      // loop through and save only the numbers to a seperate string
      while(info_file.peek() != '\n')
      {
        char c = info_file.peek();
        // check if the character is a number
        if(g_found)
        {
          if(c >= 48 && c <= 57)
          {
            ROS_INFO_STREAM(image_group << " Capture: " << c);
            digits = c + digits;
          }
        }
        else if(c == 'g')
        {
          g_found = true;
        }

        ROS_INFO_STREAM(image_group << " Capture: " << digits);

        info_file.seekg(-1, std::ios_base::cur);
      }


      // set the count number based number in the last row of the text file
      count = std::stoi(digits);

      ROS_INFO_STREAM(image_group << " Capture: Image count starting at " << count);

    }

    else
    {
      ROS_INFO_STREAM(image_group << " Capture: Using New File for Annotation.");
      info_file.clear(); // reset the image file
    }
  }

  ~ImageConverter()
  {
    info_file.close();
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

    cv::imshow("Capture for " + image_group, cv_ptr->image);

    char key = static_cast<char>(cv::waitKey(1));

    image_name = image_group + "/" + image_group + std::to_string(count) + ".jpg";


    // save the whole image
    if(key == 's')
    {

      saveImage(cv_ptr->image);

      if(generate_annotation) annotateImage();

    }

    // if(key == 'c')
    // {
    //   cv::destroyWindow("Capture for " + image_group);
    //
    //   img = cv_ptr->image.clone();
    //
    //   cropImage();
    // }
  }

  void saveImage(cv::Mat pic)
  {
    ROS_INFO_STREAM("Saved file: " << full_directory + image_name);

    cv::imwrite(full_directory + image_name, pic);

    count++;
  }

  static void mouse_callback_static(int event, int x, int y, int flag, void *param)
  {
    ImageConverter* another = static_cast<ImageConverter*>(param);
    another->mouse_callback(event, x, y, flag);
  }

  void mouse_callback(int event, int x, int y, int flag)
  {

      if (event == cv::EVENT_LBUTTONDOWN)
      {
        start_corner.x = x;
        start_corner.y = y;
      }
      else if (event == cv::EVENT_LBUTTONUP)
      {
        end_corner.x = x;
        end_corner.y = y;

        cv::rectangle(img, start_corner, end_corner, cv::Scalar(0,0,255));

        done_drawing = true;
      }
  }

  void cropImage()
  {
    cv::imshow("Crop for " + image_group, img);
    cv::setMouseCallback("Crop for " + image_group, mouse_callback_static, (void*) this);

    bool cropping = true;

    cv::Mat img_og = img.clone();

    while(cropping)
    {
      char key = cv::waitKey(1);

      if (key == 27)
      {
        cropping = false;
      }

      if (done_drawing)
      {
        cv::imshow("Crop for " + image_group, img);

        char key = cv::waitKey(0);

        if(key == 's')
        {
          cv::Rect roi;
          roi.x = std::min(start_corner.x, end_corner.x);
          roi.y = std::min(start_corner.y, end_corner.y);
          roi.width = std::max(start_corner.x, end_corner.x) - roi.x;
          roi.height = std::max(start_corner.y, end_corner.y) - roi.y;

          saveImage(img_og(roi));

          cropping = false;
        }
        else
        {
          cropping = false;
        }

      }
    }

    cv::destroyWindow("Crop for " + image_group);
    done_drawing = false;
  }

  void annotateImage()
  {
    ROS_INFO_STREAM("Annotation: " << image_name << "\n" );
    info_file << image_name << "\n";
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_capture");
  ImageConverter ic;
  ros::spin();
  return 0;
}

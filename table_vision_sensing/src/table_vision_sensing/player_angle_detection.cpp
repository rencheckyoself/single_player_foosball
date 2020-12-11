
#include "player_angle_detection.hpp"

namespace tracking
{

  PlayerDetector::PlayerDetector(PlayerDetectionParams params) : config(params)
  {
    // intialize background subtraction
    pBackSub = cv::createBackgroundSubtractorKNN(100, 1000, false);

    roi = cv::Rect(config.roi_x, config.roi_y, config.roi_width, config.roi_height);
    boundRect = cv::Rect(0,0,0,0);
  }

  RodState PlayerDetector::detectRodAngle(cv_bridge::CvImagePtr & cv_ptr)
  {
    // only use the roi for the background subtraction
    cv::Mat cropped_img = cv_ptr->image(roi).clone();

    // Get the background mask image
    pBackSub->apply(cropped_img, fgMask);

    cv::Mat detection_img;

    // Dilate and erode the image
    cv::Mat d_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11,11));
    cv::Mat e_element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
    cv::dilate(fgMask, detection_img, d_element);
    cv::erode(detection_img, detection_img, e_element);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // Find contours in the background mask
    cv::findContours(detection_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double max_area = 0;
    int max_element = -1;

    // Find the largest countor
    for( size_t i = 0; i < contours.size(); i++ )
    {
      double temp_area = cv::contourArea(contours.at(i));
      if(temp_area > max_area)
      {
        max_element = i;
        max_area = temp_area;
      }
    }

    // Only analyze a contour if one was detected and the area is over the minimum area to be a player
    if(max_element >= 0 && max_area > config.area_limit)
    {
      boundRect = cv::boundingRect(contours[max_element]);

      rod_state.boundRect = boundRect;

      setStateParams();

      // std::cout << "Area: " << max_area << "\nWidth: " << boundRect.width << "\nTop-Left X Val: " << boundRect.x << std::endl;
    }
    else
    {
      rod_state.boundRect = cv::Rect(0,0,0,0);
      // do not call setStateParams() and assume the rod is still in the same state.
    }

    return rod_state;
  }

  void PlayerDetector::setStateParams()
  {
    rod_state.rod_is_up = boundRect.width >= config.end_kick_width;
    if(rod_state.rod_is_up) rod_state.players_are_back = boundRect.x < config.end_kick_x;
  }

}

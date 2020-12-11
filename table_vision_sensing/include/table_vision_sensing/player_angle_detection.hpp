/// \file
/// \brief a class to detect the angle of a rod
#ifndef PLAYERDETECTION_INCLUDE_GUARD_HPP
#define PLAYERDETECTION_INCLUDE_GUARD_HPP

#include <string>

#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

namespace tracking
{

  /// \brief describes the state of the rod. Back is raised with feet closest to own goal, Neutral is not raised, Up is raised with feet closest to opponent goal
  enum rod_angle {BACK, NEUTRAL, UP};

  /// \brief holds the parameters for detecting the angle of a rod
  struct PlayerDetectionParams
  {
    int roi_x; ///< ROI upper left hand corner pixel x coordinate
    int roi_y; ///< ROI upper left hand corner pixel y coordinate
    int roi_width; ///< ROI pixel width
    int roi_height; ///< ROI pixel height

    int area_limit; ///< threshold to filter out non_player contours
    int end_kick_width; ///< the width of the rectangle fit to the player at the end of a kick
    int end_kick_x; ///< the top left corner x coordinate of the rectangle fit to the player at the end of a kick
  };

  /// \brief tracks the state of the rod
  struct RodState
  {
    bool rod_is_up = false; ///< true if the rod is raised so that the ball can pass under
    bool players_are_back = false; ///< true if the rod is raised with their feet closest to their own goal
    cv::Rect boundRect; ///< the bounding rectangle for the detected player contour relative to the cropped image dimensions
  };

  /// \brief class to run the vision processing to detect the
  /// \TODO Incorperate using the ball position to shift the default ROI to avoid contours merging and throwing off estimation
  class PlayerDetector
  {
  public:

    /// \brief default constructor
    PlayerDetector() {};

    /// \brief construct the angle detection with the proper parameters.
    /// \param params the parameters for detecting the angle of a rod
    PlayerDetector(PlayerDetectionParams params);

    /// \brief Function to determine the state of the rod based on the provided image
    /// \param cv_ptr pointer to the most recent image
    /// \returns the rod state
    RodState detectRodAngle(cv_bridge::CvImagePtr & cv_ptr);

  private:

    /// \brief use the bounding rectangle to determine if the rod is up and back, sets the bool values of rod_state
    void setStateParams();

    PlayerDetectionParams config;

    cv::Ptr<cv::BackgroundSubtractor> pBackSub;

    cv::Mat fgMask;

    RodState rod_state;

    cv::Rect roi; ///< the ROI of image to track a player
    cv::Rect boundRect;

  };

}

#endif

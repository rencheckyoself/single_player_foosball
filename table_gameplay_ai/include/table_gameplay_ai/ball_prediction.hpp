/// \file
/// \brief a wrapper class to track the ball in world frame coordinates and predict the trajectory
#ifndef TRACKING_INCLUDE_GUARD_HPP
#define TRACKING_INCLUDE_GUARD_HPP

#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <XmlRpcValue.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <iostream>

namespace tracking
{

  /// \brief class to track and predict the trajectory of the ball
  class BallTracker
  {
  public:

    /// \brief default contrsuctor
    BallTracker();

    /// \brief Convert the balls image position to real world coordinates
    /// \returns the xyz postion of the ball relative to the world frame defined by the points in ball_locations.yaml
    cv::Point3d getWorldPosition();

  private:

    /// \brief callback function to store the most up to date image location of the ball
    /// \param msg the ball image coordinates
    void storeBallPos(geometry_msgs::Point msg);

    ros::Subscriber ball_pos_sub; ///< subscriber for the ball position

    cv::Matx31d ball_img_pos; ///< The current ball position in image coordinates

    std::vector<cv::Point3d> world_points; ///< list of world points for known ball positions
    std::vector<cv::Point2d> image_points; ///< corresponding list of image coordinates

    cv::Mat R_w; ///< rotation matirix
    cv::Mat t_w; ///< translation vector

    cv::Matx33d intrinsic; ///< intrinsic camera matrix

    // Values to precompute for converting to world coordinates
    cv::Matx33d intrinsic_inv; ///< inverse of the intrinsic camera matrix
    cv::Mat R_w_T; ///< transposed rotation matirix
    cv::Mat M1; ///< A^-1 * R^T
    cv::Mat M2; ///< R^T * t
  };

  /// \brief helper funtion to parse through point data
  /// \param point_data data from ball_locations.yaml file
  /// \returns a vector of each point in the data
  std::vector<std::vector<double>> parse_points_data(XmlRpc::XmlRpcValue point_data);

}

#endif

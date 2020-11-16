/// \file
/// \brief a wrapper class to track the ball in world frame coordinates and predict the trajectory
#ifndef TRACKING_INCLUDE_GUARD_HPP
#define TRACKING_INCLUDE_GUARD_HPP

// #include <iostream>
#include <utility>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>

#include <XmlRpcValue.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace tracking
{

  /// \brief class to track and predict the trajectory of the ball
  class BallTracker
  {
  public:

    /// \brief default contrsuctor
    BallTracker() {};

    /// \brief provide the known z height to assume the ball is always at
    /// \param z the z coordinate of the ball in the world frame
    BallTracker(double z);

    /// \brief Convert the balls image position to real world coordinates
    /// \returns the xyz postion of the ball relative to the world frame defined by the points in ball_locations.yaml
    cv::Point3d getWorldPosition();

    /// \brief Convert the balls image position to real world coordinates
    /// \param uv the pixel coordinates to convert
    /// \returns the xyz postion of the ball relative to the world frame defined by the points in ball_locations.yaml
    cv::Point3d getWorldPosition(cv::Point2d uv);

    /// \brief Use the known coordinate pairs to test the output the the PnP estimate of the world frame transform
    /// \returns true if the estimated transform results in an error greater than error_threshold
    bool testExtrinsicResults();

    /// \brief Get the range of possible x coordinates based on the ball locations provided
    /// \returns the min/max values for the range of x coordinates where first is the minimum and second is the maximum
    std::pair<double,double> getXRange();

    /// \brief Get the range of possible y coordinates based on the ball locations provided
    /// \returns the min/max values for the range of y coordinates where first is the minimum and second is the maximum
    std::pair<double,double> getYRange();

  private:

    /// \brief Convert the ball's image position to real world coordinates
    /// \param pixel_coords the pixel coordinates to convert
    /// \returns the xyz postion corresponding to the image coordinate provided
    cv::Point3d toWorldConversion(cv::Matx31d uv);

    /// \brief callback function to store the most up to date image location of the ball
    /// \param msg the ball image coordinates
    void storeBallPos(geometry_msgs::Point msg);

    /// \brief Calculate the extrinsic matrix using Homography
    void extrinsicByHomography();

    /// \brief Calculate the extrinsic matrix using SolvePNP
    /// \TODO: Figure out why this yields very poor estimate for some reason...
    void extrinsicBySolvePNP();

    std::pair<double,double> xrange; ///<range of possible x positions
    std::pair<double,double> yrange; ///<rang of possoble y positions

    ros::Subscriber ball_pos_sub; ///< subscriber for the ball position

    cv::Matx31d ball_img_pos; ///< The current ball position in image coordinates

    std::vector<cv::Point3d> world_points3d; ///< list of world points for known ball positions
    std::vector<cv::Point2d> world_points2d; ///< list of world points for known ball positions in the same plane
    std::vector<cv::Point2d> image_points; ///< corresponding list of image coordinates

    double zw; ///< the known z coordinate for the ball, assuming it never leaves the field
    double error_threshold; ///< error threshold for the extrinsic properties calculation

    cv::Mat H_mat; ///< Homography matrix
    cv::Matx33d H_inv; ///< Inverse Homography matrix
  };

  /// \brief helper funtion to parse through point data
  /// \param point_data data from ball_locations.yaml file
  /// \returns a vector of each point in the data
  std::vector<std::vector<double>> parse_points_data(XmlRpc::XmlRpcValue point_data);

}

#endif

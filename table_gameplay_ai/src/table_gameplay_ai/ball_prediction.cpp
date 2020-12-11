
#include "ball_prediction.hpp"

namespace tracking
{

  BallTracker::BallTracker(double z, std::vector<std::vector<double>> wc, std::vector<std::vector<double>> ic, double e_thresh) : zw(z), error_threshold(e_thresh)
  {
    ros::NodeHandle n;

    xrange = std::make_pair(1000.0, -1000.0);
    yrange = std::make_pair(1000.0, -1000.0);

    // Get the max and min x and y values
    for(const auto pt : wc)
    {
      if(xrange.first > pt.at(0)) xrange.first = pt.at(0);
      else if(xrange.second < pt.at(0)) xrange.second = pt.at(0);

      if(yrange.first > pt.at(1)) yrange.first = pt.at(1);
      else if(yrange.second < pt.at(1)) yrange.second = pt.at(1);
    }

    // Convert reference points to opencv data types
    for( const auto pt : wc)
    {
      world_points3d.emplace_back(pt.at(0), pt.at(1), pt.at(2));
      world_points2d.emplace_back(pt.at(0), pt.at(1));
    }

    for(const auto im : ic)
    {
      image_points.emplace_back(im.at(0), im.at(1));
    }

    // Print Pairs
    for(unsigned int i = 0; i < world_points3d.size(); i++)
    {
      std::cout << "Coord Pair " << i << "\n\tWorld: " << world_points3d.at(i) << "\n\tImage: " << image_points.at(i);
    }

    // Calculate extrinsic properties
    extrinsicByHomography();

    ball_pos_sub = n.subscribe("BallPosition", 1, &BallTracker::storeBallPos, this);
  }

  bool BallTracker::testExtrinsicResults()
  {

    std::vector<double> tests;

    // loop through points and compare to the error threshold
    for(unsigned int i = 0; i < image_points.size(); i++)
    {
      cv::Point3d res = getWorldPosition(image_points.at(i));

      cv::Point3d buf = world_points3d.at(i) - res;

      double norm = cv::norm(buf);

      if(norm > error_threshold)
      {
        ROS_WARN_STREAM("Homography Error exceeds threshold for point " << i << ". Error is " << norm);
        return true;
      }
    }
    return false;
  }

  void BallTracker::storeBallPos(geometry_msgs::Point msg)
  {
    // Only update the postion if an object has been detected.
    if(msg.x != 0 && msg.y !=0) ball_img_pos = cv::Matx31d(msg.x, msg.y, 1.0);
  }

  cv::Point3d BallTracker::getWorldPosition(cv::Point2d uv)
  {
    cv::Matx31d buf;
    buf(0,0) = uv.x;
    buf(1,0) = uv.y;
    buf(2,0) = 1.0;
    return toWorldConversion(buf);
  }

  cv::Point3d BallTracker::getWorldPosition()
  {
    return toWorldConversion(ball_img_pos);
  }

  std::pair<double,double> BallTracker::getXRange()
  {
    return xrange;
  }

  std::pair<double,double> BallTracker::getYRange()
  {
    return yrange;
  }

  cv::Point3d BallTracker::toWorldConversion(cv::Matx31d uv)
  {
    cv::Point3d xyz;

    cv::Matx31d M1 = H_inv * uv;

    double s = 1/M1(2,0);

    xyz.x = M1(0,0) * s;
    xyz.y = M1(1,0) * s;
    xyz.z = zw;

    return xyz;
  }

  void BallTracker::extrinsicByHomography()
  {
    H_mat = cv::findHomography(world_points2d, image_points);

    cv::Mat buf = H_mat.inv();

    H_inv = cv::Matx33d(buf.at<double>(0,0), buf.at<double>(0,1), buf.at<double>(0,2),
                        buf.at<double>(1,0), buf.at<double>(1,1), buf.at<double>(1,2),
                        buf.at<double>(2,0), buf.at<double>(2,1), buf.at<double>(2,2));
  }

  void BallTracker::extrinsicBySolvePNP()
  {
    // std::vector<double> rvec;
    // cv::Mat rvec;
    // cv::Mat R_guess = cv::Mat::zeros(3, 3, CV_64FC1);;
    // R_guess.at<double>(0,0) = 1;
    // R_guess.at<double>(1,1) = 1;
    // R_guess.at<double>(2,2) = -1;
    // //
    // cv::Rodrigues(R_guess, rvec);
    //
    // t_w(0,0) = translation_est.at(0);
    // t_w(1,0) = translation_est.at(1);
    // t_w(2,0) = translation_est.at(2);
    //
    // std::cout << "GUESSES: \n";
    // std::cout << "Rotation: \n" << rvec;
    // std::cout << "Translation: \n" << t_w;

    // Get R and t to convert to the world frame
    // using rectified image, so do not need distortion params

    // std::cout << "#World: " << world_points3d.size() << " #Image: " << image_points.size();
    // cv::solvePnP(world_points3d, image_points, intrinsic, cv::Mat(), rvec, t_w, false);
    //
    // cv::Rodrigues(rvec, R_w);
    //
    // std::cout << R_w << std::endl;
    // std::cout << t_w << std::endl;
  }

  std::vector<std::vector<double>> parse_points_data(XmlRpc::XmlRpcValue point_data)
  {
    std::vector<std::vector<double>> output;

    for(int i = 0; i < point_data.size(); i++)
    {
      std::vector<double> p;

      for(int j = 0; j < point_data[i].size(); j++)
      {
        p.push_back(double(point_data[i][j]));
      }
      output.push_back(p);
    }

    return output;
  }
}

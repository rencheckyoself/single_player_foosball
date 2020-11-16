
#include "ball_prediction.hpp"

namespace tracking
{

  BallTracker::BallTracker(double z) : zw(z)
  {

    ros::NodeHandle n;
    ros::NodeHandle np("~");

    XmlRpc::XmlRpcValue world_coordinates;
    XmlRpc::XmlRpcValue image_coordinates;
    std::vector<double> camera_matrix;

    np.getParam("world_coordinates", world_coordinates);
    np.getParam("image_coordinates", image_coordinates);

    np.getParam("camera_matrix/data", camera_matrix);

    std::vector<std::vector<double>> wc = parse_points_data(world_coordinates);
    std::vector<std::vector<double>> ic = parse_points_data(image_coordinates);

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

    for(unsigned int i = 0; i < world_points3d.size(); i++)
    {
      ROS_INFO_STREAM("Coord Pair " << i << "\n\tWorld: " << world_points3d.at(i) << "\n\tImage: " << image_points.at(i));
    }


    // get intrinsic properties from calibration
    intrinsic = cv::Matx33d(camera_matrix.at(0), 0, camera_matrix.at(2),
                            0, camera_matrix.at(4), camera_matrix.at(5),
                            0, 0, 1);

    intrinsic_inv = intrinsic.inv();

    // extrinsicBySolvePNP();
    extrinsicByHomography();

    ROS_INFO_STREAM("WORLD RELATIVE TO THE CAMERA:\n");
    ROS_INFO_STREAM("Rotation Det, " << cv::determinant(R_w) << " for: \n" << R_w);
    ROS_INFO_STREAM("Translation: \n" << t_w);

    R_w_T = R_w.t();

    AR_T = intrinsic_inv * R_w_T;
    M2 = R_w_T * t_w;

    // ROS_INFO_STREAM("M2: \n" << M2);

    ball_pos_sub = n.subscribe("BallPosition", 1, &BallTracker::storeBallPos, this);
  }

  bool BallTracker::testExtrinsicResults()
  {

    std::vector<double> tests;

    for(unsigned int i = 0; i < image_points.size(); i++)
    {
      // cv::Point3d res = getWorldPosition(image_points.at(i));
      cv::Point3d res = getWorldPositionHomog(image_points.at(i));

      cv::Point3d buf = world_points3d.at(i) - res;

      ROS_INFO_STREAM("Test error for point " << i << ": " << buf);

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

  cv::Point3d BallTracker::getWorldPositionHomog(cv::Point2d uv)
  {
    cv::Matx31d buf;
    buf(0,0) = uv.x;
    buf(1,0) = uv.y;
    buf(2,0) = 1.0;
    return toWorldConversionHomog(buf);
  }

  cv::Point3d BallTracker::getWorldPosition()
  {
    return toWorldConversion(ball_img_pos);
  }

  cv::Point3d BallTracker::getWorldPositionHomog()
  {
    return toWorldConversionHomog(ball_img_pos);
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

    cv::Matx31d M1 = AR_T * uv;

    double s = calc_s(M1);

    xyz.x = M1(0,0) * s - M2(0,0);
    xyz.y = M1(1,0) * s - M2(1,0);
    // xyz.y *= -1; // Hack to fix SolvePnP
    xyz.z = zw;

    return xyz;
  }

  cv::Point3d BallTracker::toWorldConversionHomog(cv::Matx31d uv)
  {
    cv::Point3d xyz;

    cv::Matx31d M1 = H_inv * uv;

    double s = 1/M1(2,0);

    xyz.x = M1(0,0) * s;
    xyz.y = M1(1,0) * s;
    xyz.z = 0.0;

    return xyz;
  }

  double BallTracker::calc_s(cv::Matx31d M1)
  {
    return (zw + M2(2,0))/M1(2,0);
  }

  void BallTracker::extrinsicByHomography()
  {
    H_mat = cv::findHomography(world_points2d, image_points);

    cv::Mat buf = H_mat.inv();

    H_inv = cv::Matx33d(buf.at<double>(0,0), buf.at<double>(0,1), buf.at<double>(0,2),
                        buf.at<double>(1,0), buf.at<double>(1,1), buf.at<double>(1,2),
                        buf.at<double>(2,0), buf.at<double>(2,1), buf.at<double>(2,2));

    double norm = sqrt(H_mat.at<double>(0,0)*H_mat.at<double>(0,0) + H_mat.at<double>(1,0)*H_mat.at<double>(1,0) + H_mat.at<double>(2,0)*H_mat.at<double>(2,0));

    H_mat /= norm;
    cv::Mat c1  = H_mat.col(0);
    cv::Mat c2  = H_mat.col(1);
    cv::Mat c3 = c1.cross(c2);
    cv::Mat t = H_mat.col(2);

    t_w(0,0) = t.at<double>(0,0);
    t_w(1,0) = t.at<double>(1,0);
    t_w(2,0) = 0.300; // measured distance from board to lens, doesnt matter for points on the field tho b/c they are at zw=0

    cv::Mat R(3, 3, CV_64F);

    for (int i = 0; i < 3; i++)
    {
        R.at<double>(i,0) = c1.at<double>(i,0);
        R.at<double>(i,1) = c2.at<double>(i,0);
        R.at<double>(i,2) = c3.at<double>(i,0);
    }

    std::cout << "R (before polar decomposition):\n" << R << "\ndet(R): " << cv::determinant(R) << std::endl;
    cv::Mat W, U, Vt;
    cv::SVDecomp(R, W, U, Vt);
    R = U*Vt;
    std::cout << "R (after polar decomposition):\n" << R << "\ndet(R): " << cv::determinant(R) << std::endl;

    R_w = cv::Matx33d(R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                      R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                      R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));

  }

  void BallTracker::extrinsicBySolvePNP()
  {
    std::vector<double> rvec;
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
    // ROS_INFO_STREAM("GUESSES: \n");
    // ROS_INFO_STREAM("Rotation: \n" << rvec);
    // ROS_INFO_STREAM("Translation: \n" << t_w);

    // Get R and t to convert to the world frame
    // using rectified image, so do not need distortion params

    ROS_INFO_STREAM("#World: " << world_points3d.size() << " #Image: " << image_points.size());
    cv::solvePnP(world_points3d, image_points, intrinsic, cv::Mat(), rvec, t_w, false);

    cv::Rodrigues(rvec, R_w);

    std::cout << R_w << std::endl;
    std::cout << t_w << std::endl;

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

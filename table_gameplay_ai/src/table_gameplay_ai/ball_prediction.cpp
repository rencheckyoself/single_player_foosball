
#include "ball_prediction.hpp"

namespace tracking
{

  BallTracker::BallTracker()
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

    // Convert reference points to opencv data types
    for( const auto pt : wc)
    {
      world_points.emplace_back(pt.at(0), pt.at(1), pt.at(2));
    }

    for(const auto im : ic)
    {
      image_points.emplace_back(im.at(0), im.at(1));
    }

    for(unsigned int i = 0; i < world_points.size(); i++)
    {
      ROS_INFO_STREAM("Coord Pair " << i << "\n\tWorld: " << world_points.at(i) << "\n\tImage: " << image_points.at(i));
    }

    // get intrinsic properties from calibration
    intrinsic = cv::Matx33d(camera_matrix.at(0), 0, camera_matrix.at(2),
                            0, camera_matrix.at(4), camera_matrix.at(5),
                            0, 0, 1);

    std::vector<double> rvec;
    t_w.create(3, 1, cv::DataType<double>::type);
    // std::vector<double> tvec;

    // Get R and t to convert to the world frame
    // using rectified image, so do not need distortion params
    cv::solvePnP(world_points, image_points, intrinsic, 0, rvec, t_w);

    cv::Rodrigues(rvec, R_w);

    ROS_INFO_STREAM("Rotation: " << R_w);
    ROS_INFO_STREAM("Translation: " << t_w);

    intrinsic_inv = intrinsic.inv();
    R_w_T = R_w.clone().t();

    left = intrinsic_inv * R_w_T;
    right = R_w_T * t_w;

    ball_pos_sub = n.subscribe("BallPosition", 1, &BallTracker::storeBallPos, this);
  }

  void BallTracker::storeBallPos(geometry_msgs::Point msg)
  {
    ball_img_pos = cv::Matx31d(msg.x, msg.y, 1);
  }

  cv::Point3d BallTracker::getWorldPosition()
  {
    cv::Point3d xyz;

    double s = s();

    return xyz;
  }

  std::vector<std::vector<double>> parse_points_data(XmlRpc::XmlRpcValue point_data)
  {
    std::vector<std::vector<double>> output;

    for(int i = 0; i < point_data.size(); i++)
    {
      auto point = point_data[i];
      std::vector<double> p;
      for(int j = 0; j < point.size(); j++)
      {
        p.push_back(point[j]);
      }
      output.push_back(p);
    }

    return output;
  }
}

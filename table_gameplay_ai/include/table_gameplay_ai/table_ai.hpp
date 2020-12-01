/// \file
/// \brief a wrapper class to track the ball in world frame coordinates and predict the trajectory
#ifndef TABLEAI_INCLUDE_GUARD_HPP
#define TABLEAI_INCLUDE_GUARD_HPP

#include <utility>
#include <algorithm>
#include <cmath>

#include <opencv2/highgui/highgui.hpp>
#include "table_motor_control/tic_server.hpp"


namespace table
{

  /// \brief struct to hold all of the tic controller info
  struct ControllerInfo
  {
    std::string fwd_rot_sn, fwd_lin_sn, def_rot_sn, def_lin_sn; ///< serial numbers for tic controllers
    std::string fwd_rot_nickname, fwd_lin_nickname, def_rot_nickname, def_lin_nickname; ///< nicknames for the tic controllers

    /// \brief default constructor
    ControllerInfo() {};
  };

  /// \brief holds the parameters for detecting the angle of a rod
  struct PlayerDetection
  {
    int roi_x; ///< ROI upper left hand corner pixel x coordinate
    int roi_y; ///< ROI upper left hand corner pixel y coordinate
    int roi_width; ///< ROI pixel width
    int roi_height; ///< ROI pixel height

    int area_limit; ///< threshold to filter out non_player contours
    int end_kick_width; ///< the width of the rectangle fit to the player at the end of a kick
    int end_kick_x; ///< the top left corner x coordinate of the rectangle fit to the player at the end of a kick
  };

  /// \brief struct to hold all of the calibration params for the table
  struct CalibrationVals
  {
    std::pair<double,double> xrange; ///< first: min, second: max, the range of possible x values for the ball
    std::pair<double,double> yrange; ///< first: min, second: max, the range of possible y values for the ball

    std::pair<double,double> lin_joint_limits; ///< first: lower, second: upper, linear joint limits from urdf

    std::pair<int, int> lin_step_limits; ///< first: lower, second: upper, the absolute stepper positions corresponding to the lin_joint_limits

    int linear_hysteresis; ///< the hysteresis threshold for the linear position. The smaller the value, the more sensative the command generate will be

    double kick_trigger_threshold; ///< defines the +/- threshold for initiating a kick

    double def_rod_xpos; ///< the x value (in world coordinates) of the ideal position for the defensive rod to strike the ball
    double fwd_rod_xpos; ///< the x value (in world coordinates) of the ideal position for the defensive rod to strike the ball

    PlayerDetection fwd_rod; ///< angle detection params for the offensive rod
    PlayerDetection def_rod; ///< angle detection params for the defensive rod
  };

  /// \brief a class to generate a control based on the state of the table
  class FoosballTable
  {

  public:

    /// \brief Default constructor
    FoosballTable() {};

    /// \brief Pass the config parameters to the FoosballTable
    /// \param table_config congifuration parameters
    FoosballTable(CalibrationVals table_config);

    /// \brief default destructor
    virtual ~FoosballTable() {};

    /// \brief move the table based on the ball's position
    /// \param ball_pos the current ball position
    virtual void moveTable(cv::Point3d ball_pos);

    /// \brief get the position of the motors
    /// returns the current joint positions
    std::vector<double> getCurrentJointStates();

  protected:

    /// \brief generate a Joint State message based on the ball position
    /// \param ball_pos the current ball position
    void generateJointStates(cv::Point3d ball_pos);

    /// \brief generate an absolute linear position (in meters) to move the rod to based on the y position of the ball
    /// \param pos the current y position of the ball
    double getLinearPosition(double pos);

    /// \brief generate an absolute angular position (in radians) to move the rod to based on the x position of the ball
    /// \param ball_pos the current ball position
    /// \param rod_x_coord the x position of the ideal kicking location a rod
    double getAngularPosition(double ball_pos, double rod_x_coord);

    /// \brief Map a value from one range to another
    /// \param input the value to map
    /// \param from_min the minimum of the range you are mapping from
    /// \param from_max the maximum of the range you are mapping from
    /// \param to_min the minimum of the range you are mapping to
    /// \param to_max the maximum of the range you are mapping to
    /// \returns the remapped input value
    /// \TODO: add bounds checking on the input value
    double map_ranges(double input, double from_min, double from_max, double to_min, double to_max);

    double current_lin_pos = 0; ///< the current linear position of the rods

    std::vector<double> joint_states; ///< joint states to publish: [def_ang, def_lin, fwd_ang, fwd_lin, 0, 0, 0, 0]

    CalibrationVals config; ///< all of the configuration parameters to properly generate the controls
  };

  class RealFoosballTable : public FoosballTable
  {

  public:

    /// \brief default constructor
    // RealFoosballTable() {};

    /// \brief default destructor
    ~RealFoosballTable();

    /// \brief initialize the table with the physical conrtollers
    /// \param tic_info struct containing all of the info to initialize the tic controllers.
    /// \param table_config congifuration parameters
    RealFoosballTable(ControllerInfo tic_info, CalibrationVals table_config);

    /// \brief generate a joint state message based on the ball position and issue commands to the tic controllers
    /// \param ball_pos the current ball position
    void moveTable(cv::Point3d ball_pos);

    /// \brief deenergize the entire table
    void deenergize();

    /// \brief energize the entire table
    void energize();

  private:

    /// \brief convert the joint states into absolute stepper positions and send the values to the tic controllers
    void issueStepperControls();

    tic_server::TicCtrlr def_rot;
    tic_server::TicCtrlr def_lin;
    tic_server::TicCtrlr fwd_rot;
    tic_server::TicCtrlr fwd_lin;
  };
}

#endif

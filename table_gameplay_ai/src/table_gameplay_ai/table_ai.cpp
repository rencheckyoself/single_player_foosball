
#include "table_ai.hpp"

namespace table
{

  //=====================================================================================================================
  //======================================== FoosballTable ==============================================================
  //=====================================================================================================================

  FoosballTable::FoosballTable(CalibrationVals table_config) : config(table_config)
  {
    joint_states = std::vector<double>(8,0);
  }

  void FoosballTable::moveTable(cv::Point3d ball_pos)
  {
    generateJointStates(ball_pos);
  }

  std::vector<double> FoosballTable::getCurrentJointStates()
  {
    return joint_states;
  }

  void FoosballTable::generateJointStates(cv::Point3d ball_pos)
  {
    // set linear positions
    joint_states.at(1) = getLinearPosition(ball_pos.y);
    joint_states.at(3) = joint_states.at(0);

    // set angular positions
    joint_states.at(0) = getAngularPosition(ball_pos.x, config.def_rod_xpos);
    joint_states.at(2) = getAngularPosition(ball_pos.x, config.fwd_rod_xpos);
  }

  double FoosballTable::getLinearPosition(double pos)
  {
    if(!std::isnan(pos))
    {
      double third_distance = config.yrange.second / 3.0;

      if(pos > third_distance && pos <= 2*third_distance) pos -= third_distance;
      else if(pos > 2*third_distance) pos -= (2*third_distance);

      current_lin_pos = map_ranges(pos, config.yrange.first, config.yrange.first + third_distance, config.lin_joint_limits.first, config.lin_joint_limits.second);
    }

    return std::clamp(current_lin_pos, config.lin_joint_limits.first, config.lin_joint_limits.second);
  }

  double FoosballTable::getAngularPosition(double pos, double rod_x_coord)
  {
    // test if the position is valid
    if(!std::isnan(pos))
    {
      if(pos > rod_x_coord - config.kick_trigger_threshold)
      {
        if(pos > rod_x_coord + config.kick_trigger_threshold) return 0.5; // return an angle slightly back from vertical
        else return -1.5; // tell the table to kick
      }
      else return 1.5; // fully raise players
    }
    else return 0; // position the players to be vertical
  }

  double FoosballTable::map_ranges(double input, double from_min, double from_max, double to_min, double to_max)
  {
    double ratio = (to_max - to_min) / (from_max - from_min);
    return to_min + ratio * (input - from_min);
  }

  //=====================================================================================================================
  //======================================== RealFoosballTable ==========================================================
  //=====================================================================================================================

  RealFoosballTable::~RealFoosballTable()
  {
    deenergize();
  }


  RealFoosballTable::RealFoosballTable(ControllerInfo tic_info, CalibrationVals table_config) : FoosballTable(table_config)
  {

    def_rot = tic_server::TicCtrlr(tic_info.def_rot_sn, tic_info.def_rot_nickname);
    def_lin = tic_server::TicCtrlr(tic_info.def_lin_sn, tic_info.def_lin_nickname);

    fwd_rot = tic_server::TicCtrlr(tic_info.fwd_rot_sn, tic_info.fwd_rot_nickname);
    fwd_lin = tic_server::TicCtrlr(tic_info.fwd_lin_sn, tic_info.fwd_lin_nickname);

    RealFoosballTable::energize();
  }

  void RealFoosballTable::moveTable(cv::Point3d ball_pos)
  {
    generateJointStates(ball_pos);

    issueStepperControls();
  }

  void RealFoosballTable::issueStepperControls()
  {
    int lin_pos = std::floor(map_ranges(joint_states.at(1), config.lin_joint_limits.first, config.lin_joint_limits.second, config.lin_step_limits.first, config.lin_step_limits.second));

    // Issue the linear command if allowed by hysteresis setting
    if(std::abs(lin_pos - def_lin.get_current_pos()) > config.linear_hysteresis)
    {
      def_lin.set_position(lin_pos);
      fwd_lin.set_position(lin_pos);
    }

    int def_rot_stepper = std::floor(map_ranges(joint_states.at(0), -3.14159, 3.14159, -100, 100));
    int fwd_rot_stepper = std::floor(map_ranges(joint_states.at(2), -3.14159, 3.14159, -100, 100));

    def_rot.set_position(def_rot_stepper);
    fwd_rot.set_position(fwd_rot_stepper);
  }

  void RealFoosballTable::energize()
  {
    // enable and energize the motors
    def_rot.resume();
    def_lin.resume();

    fwd_rot.resume();
    fwd_lin.resume();

    // set the current position to 0
    def_rot.reset_global_position();
    def_lin.reset_global_position();

    fwd_rot.reset_global_position();
    fwd_lin.reset_global_position();
  }

  void RealFoosballTable::deenergize()
  {
    fwd_rot.deenergize();
    fwd_lin.deenergize();

    def_rot.deenergize();
    def_lin.deenergize();
  }
}

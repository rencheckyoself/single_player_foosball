
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

  void FoosballTable::moveTable(cv::Point3d ball_pos, tracking::RodState, tracking::RodState)
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
    joint_states.at(3) = joint_states.at(1);

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
      // check if the ball is within the threshold
      if(pos > rod_x_coord - (3*config.kick_trigger_threshold) && pos < rod_x_coord + config.kick_trigger_threshold)
      {
        return -1.5;
      }
      else if(pos > rod_x_coord + config.kick_trigger_threshold) return 0.5; // return an angle slightly back from vertical
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

  void RealFoosballTable::moveTable(cv::Point3d ball_pos, tracking::RodState, tracking::RodState)
  {
    generateJointStates(ball_pos);

    kickCheck();

    issueStepperControls();
  }

  void RealFoosballTable::kickCheck()
  {

    // If the stepper is at the vertical position, be done resetting
    if(getStepperRotVal(0.5) == def_rot.get_current_pos()) def_reseting = false;

    // If the stepper is in the process of resetting, continue to go there
    if(def_reseting)
    {
      joint_states.at(0) = 0.5;
    }
    // If the stepper is at the end of the kick, start resetting
    else if(getStepperRotVal(-1.5) == def_rot.get_current_pos())
    {
      def_reseting = true;
      joint_states.at(0) = 0.5;
    }

    if(getStepperRotVal(0.5) == fwd_rot.get_current_pos()) fwd_reseting = false;

    if(fwd_reseting)
    {
      joint_states.at(2) = 0.5;
    }
    else if(getStepperRotVal(-1.5) == fwd_rot.get_current_pos())
    {
      fwd_reseting = true;
      joint_states.at(2) = 0.5;
    }

  }

  void RealFoosballTable::issueStepperControls()
  {
    sendLinearControl();

    sendRotationalControl();
  }

  void RealFoosballTable::sendLinearControl()
  {
    int lin_pos = std::floor(map_ranges(joint_states.at(1), config.lin_joint_limits.first, config.lin_joint_limits.second, config.lin_step_limits.first, config.lin_step_limits.second));

    // Issue the linear command if allowed by hysteresis setting
    if(std::abs(lin_pos - def_lin.get_current_pos()) > config.linear_hysteresis)
    {
      def_lin.set_position(lin_pos);
      fwd_lin.set_position(lin_pos);
    }
  }

  void RealFoosballTable::sendRotationalControl()
  {
    int def_rot_stepper = getStepperRotVal(joint_states.at(0));
    int fwd_rot_stepper = getStepperRotVal(joint_states.at(2));

    def_rot.set_position(def_rot_stepper);
    fwd_rot.set_position(fwd_rot_stepper);
  }

  int RealFoosballTable::getStepperRotVal(double input)
  {
    return std::floor(map_ranges(input, -3.14159, 3.14159, -100, 100));
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

  //=====================================================================================================================
  //======================================== DeathSpinTable =============================================================
  //=====================================================================================================================

  void DeathSpinTable::moveTable(cv::Point3d ball_pos, tracking::RodState, tracking::RodState)
  {
    joint_states.at(1) = getLinearPosition(ball_pos.y);
    joint_states.at(3) = joint_states.at(1);

    issueStepperControls();
  }

  void DeathSpinTable::sendRotationalControl()
  {
    def_rot.set_velocity(-std::floor(config.velocity_modifier * def_rot.get_max_speed()));
    fwd_rot.set_velocity(-std::floor(config.velocity_modifier * def_rot.get_max_speed()));
  }

  //=====================================================================================================================
  //======================================== FeedbackTable ==============================================================
  //=====================================================================================================================


  void FeedbackTable::moveTable(cv::Point3d ball_pos, tracking::RodState fwd_rod_state, tracking::RodState def_rod_state)
  {
    generateJointStates(ball_pos);

    kickCheck(fwd_rod_state, def_rod_state);

    issueStepperControls();
  }


  void FeedbackTable::sendRotationalControl()
  {
    if(def_reseting)
    {
      def_rot.set_position(def_end_reset_pos);
    }
    else if(!def_kicking)
    {
      if(joint_states.at(0) == -1.5)
      {
        def_kick_start_pos = def_rot.get_current_pos();
        def_rot.set_velocity(-std::floor(config.velocity_modifier * def_rot.get_max_speed()));
        def_kicking = true;
      }
      // else if(joint_states.at(0) == 1.5)
      // {
      //   def_kick_start_pos = def_rot.get_current_pos();
      //   def_rot.set_velocity(std::floor(config.velocity_modifier * def_rot.get_max_speed()));
      //   def_kicking = true;
      // }
      else
      {
        def_rot.set_position(getStepperRotVal(joint_states.at(0)));
      }
    }

    if(fwd_reseting)
    {
      fwd_rot.set_position(fwd_end_reset_pos);
    }
    else if(!fwd_kicking)
    {
      if(joint_states.at(2) == -1.5)
      {
        fwd_kick_start_pos = fwd_rot.get_current_pos();
        fwd_rot.set_velocity(-std::floor(config.velocity_modifier * fwd_rot.get_max_speed()));
        fwd_kicking = true;
      }
      else
      {
        fwd_rot.set_position(getStepperRotVal(joint_states.at(2)));
      }
    }
  }

  void FeedbackTable::kickCheck(tracking::RodState fwd_rod_state, tracking::RodState def_rod_state)
  {

    tracking::rod_angle def_rod, fwd_rod;

    // Determine the state of the defensive rod
    if(def_rod_state.rod_is_up)
    {
      if(def_rod_state.players_are_back) def_rod = tracking::BACK;
      else def_rod = tracking::UP;
    }
    else
    {
      def_rod = tracking::NEUTRAL;
    }

    // Determine the state of the offensive rod
    if(fwd_rod_state.rod_is_up)
    {
      if(fwd_rod_state.players_are_back) fwd_rod = tracking::BACK;
      else fwd_rod = tracking::UP;
    }
    else
    {
      fwd_rod = tracking::NEUTRAL;
    }

    // Check if the kicking or resetting action has been completed.
    if(def_kicking)
    {
      // stop rotating if the the rod is up or has completed 1 full rotation.
      if(def_rod == tracking::UP || def_rot.get_current_pos() <= def_kick_start_pos - config.full_rotation_offset)
      {
        def_rot.halt();
        def_kicking = false;
        def_reseting = true;
        def_reset_dir = 1;
        def_end_reset_pos = def_rot.get_current_pos() + config.def_reset_offset; // set the reseting target position to be slight larger than the 90deg.
        joint_states.at(0) = 0.5;
      }
      // stop rotating if the the rod is back or has completed 1 full rotation.
      // else if(def_rod == tracking::BACK || def_rot.get_current_pos() >= def_kick_start_pos + config.full_rotation_offset)
      // {
      //   def_rot.halt();
      //   def_kicking = false;
      //   def_reseting = true;
      //   def_reset_dir = -1;
      //   def_end_reset_pos = def_rot.get_current_pos() - config.def_reset_offset; // set the reseting target position to be slight larger than the 90deg.
      //   joint_states.at(0) = 0.5;
      // }
    }
    else if(def_reseting)
    {
      joint_states.at(0) = 0.5;

      // stop reseting if the target has been reached or exceeded
      if( def_reset_dir * (def_rot.get_current_pos() - def_end_reset_pos) >= 0)
      {
        def_reseting = false;
        def_rot.reset_global_position();
      }
    }

    if(fwd_kicking)
    {
      if(fwd_rod == tracking::UP || fwd_rot.get_current_pos() <= fwd_kick_start_pos - config.full_rotation_offset)
      {
        fwd_rot.halt();
        fwd_kicking = false;
        fwd_reseting = true;
        fwd_end_reset_pos = fwd_rot.get_current_pos() + config.fwd_reset_offset;
        joint_states.at(2) = 0.5;
      }
    }
    else if(fwd_reseting)
    {
      joint_states.at(2) = 0.5;

      if(fwd_rot.get_current_pos() >= fwd_end_reset_pos)
      {
        fwd_reseting = false;
        fwd_rot.reset_global_position();
      }
    }
  }
}

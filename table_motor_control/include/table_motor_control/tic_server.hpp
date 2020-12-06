/// \file
/// \brief a wrapper class to interact with Tic C++ API and manage service callbacks
#ifndef TIC_SERVER_INCLUDE_GUARD_HPP
#define TIC_SERVER_INCLUDE_GUARD_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <std_srvs/Empty.h>

#include <stdint.h>
#include <iostream>
#include <string>

#include "tic.hpp"
#include "table_motor_control/Int32.h"

namespace tic_server
{

  /// \brief Class to wrap some of the tic API with ROS functionality
  class TicCtrlr
  {
  public:

    /// \brief default constructor
    TicCtrlr() {};

    /// \brief Main constructor
    /// \param desired_serial_number string of numbers to uniquely identify a tic controller
    /// \param name a unique readable nickename for the tic controller
    TicCtrlr(std::string desired_serial_number, std::string name);

    /// \brief Initialize all of the services to offer expose functionality to user
    void offer_services();

    /// \brief get the current position of the stepper
    /// \returns the integer position for where the controller thinks the stepper is
    int32_t get_current_pos();

    /// \brief callback service wrapper function to send the motor to a target position
    /// \param req service request data is only an int32
    /// \param res no response
    /// \returns 1 for success, 0 for failure
    bool set_position(table_motor_control::Int32::Request &req, table_motor_control::Int32::Response &res);

    /// \brief send the stepper a target position
    /// \param val an absolute position value in number of steps to move.
    void set_position(int32_t val);

    /// \brief callback service wrapper function to send the motor to a target velocity
    /// \param req service request data is only an int32
    /// \param res no response
    /// \returns 1 for success, 0 for failure
    bool set_velocity(table_motor_control::Int32::Request &req, table_motor_control::Int32::Response &);

    /// \brief send the stepper a target position
    /// \param val velocity in pulses/s to move.
    void set_velocity(int32_t val);

    /// \brief callback service wrapper function to reset the 0 absolute position
    /// \param req no request
    /// \param res no response
    /// \returns 1 for success, 0 for failure
    bool reset_global_position(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /// \brief stops the motor and sets 0 absolute position to the current position.
    void reset_global_position();

    /// \brief prints the current settings of the tic controller
    void display_settings();

    /// \brief callback service wrapper function to import settings
    /// \param req no request
    /// \param res no response
    /// \returns 1 for success, 0 for failure
    bool import_settings(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /// \brief imports a text file to change the settings of the tic controller
    void import_settings();

    /// \brief callback service wrapper function to resume stepped operation
    /// \param req no request
    /// \param res no response
    /// \returns 1 for success, 0 for failure
    bool resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /// \brief energizes the stepper motor and exits the safe start state
    void resume();

    /// \brief callback service wrapper function to resume stepped operation
    /// \param req no request
    /// \param res no response
    /// \returns 1 for success, 0 for failure
    bool deenergize(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

    /// \brief cuts off current to the motor so there is no holding power
    void deenergize();

  private:

    /// \brief Opens a handle to a Tic that can be used for communication. Code copied from Tic User Manual.
    /// \param desired_serial_number serial number of TIC board
    /// returns the handle to the tic board
    tic::handle open_handle(std::string desired_serial_number);

    std::string nickname; ///< unique, readable identifier for this motor
    std::string serial_number; ///< serial number for the control board
    tic::handle handle; ///< handle to interact with the Tic API

    ros::ServiceServer settings_service; ///< service to update a tic controllers settings
    ros::ServiceServer target_service; ///< service to send an absolute position command
    ros::ServiceServer reset_home_service; ///< stops the motor and sets the current position as the 0 position
    ros::ServiceServer resume_service; ///< turns the motor on
    ros::ServiceServer deenergize_service; ///< turns the motor off

    ros::NodeHandle n; ///< node handle to offer services.
  };
}

#endif //TIC_SERVER_INCLUDE_GUARD_HPP

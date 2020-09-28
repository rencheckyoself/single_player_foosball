/// \file
/// \brief a wrapper class to interact with Tic C++ API and manage service callbacks
#ifndef TIC_SERVER_INCLUDE_GUARD_HPP
#define TIC_SERVER_INCLUDE_GUARD_HPP

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <stdint.h>
#include <iostream>
#include <tic.hpp>

namespace tic_server
{

  /// \brief Class to wrap some of the tic API with ROS functionality
  class TicCtrlr
  {
  public:

    /// \brief Main constructor
    /// \param desired_serial_number string of numbers to uniquely identify a tic controller
    /// \param name a unique readable nickename for the tic controller
    TicCtrlr(std::string desired_serial_number, std::string name);

    /// \brief Initialize all of the services to offer expose functionality to user and other nodes
    /// \param n reference to a node handle
    void OfferServices(ros::NodeHandle &n);

    /// \brief get the current position of the stepper
    void get_current_pos();

    /// \brief send the stepper a target position
    /// \param val an absolute position value in number of steps to move.
    void set_position(int32_t val);

    /// \brief prints the current settings of the tic controller
    void display_settings();

    /// \brief imports a text file to change the settings of the tic controller
    bool import_settings(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  private:

    /// \brief Opens a handle to a Tic that can be used for communication. Code pulled from Tic User Manual.
    ///
    /// To open a handle to any Tic:
    ///   tic_handle * handle = open_handle();
    /// To open a handle to the Tic with serial number 01234567:
    ///   tic_handle * handle = open_handle("01234567");
    /// \param desired_serial_number serial number of TIC board
    /// returns the handle to the tic board
    tic::handle open_handle(std::string desired_serial_number);

    std::string nickname; ///< unique, readable identifier for this motor
    std::string serial_number; ///< serial number for the control board
    tic::handle handle; ///< handle to interact with the Tic API

    ros::ServiceServer service;
  };

}

#endif //PFIELD_INCLUDE_GUARD_HPP

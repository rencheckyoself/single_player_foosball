/// \file
/// \brief a wrapper class to interact with Tic C++ API and manage service callbacks
#ifndef TIC_SERVER_INCLUDE_GUARD_HPP
#define TIC_SERVER_INCLUDE_GUARD_HPP

#include <ros/ros.h>

#include <iostream>
#include <tic.hpp>

namespace tic_server
{

  class TicCtrlr
  {
  public:

    TicCtrlr(std::string desired_serial_number);

    void get_current_pos();

    void display_settings();

    void import_settings();

  private:

    /// \brief Opens a handle to a Tic that can be used for communication. Code pulled from Tic User Manual
    ///
    /// To open a handle to any Tic:
    ///   tic_handle * handle = open_handle();
    /// To open a handle to the Tic with serial number 01234567:
    ///   tic_handle * handle = open_handle("01234567");
    /// \param desired_serial_number serial number of TIC board
    /// returns the handle to the tic board
    tic::handle open_handle(std::string desired_serial_number);

    std::string serial_number; ///< serial number for the control board
    tic::handle handle; ///< handle to interact with the API
  };

}

#endif //PFIELD_INCLUDE_GUARD_HPP

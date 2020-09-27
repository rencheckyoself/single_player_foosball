/// \file
/// \brief a wrapper class to interact with Tic C++ API and manage service callbacks

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <tic.hpp>

#include "tic_server.hpp"

namespace tic_server
{
  TicCtrlr::TicCtrlr(std::string desired_serial_number)
  {
    handle = TicCtrlr::open_handle(desired_serial_number);
    serial_number = desired_serial_number;
  }

  void TicCtrlr::get_current_pos()
  {
    int32_t position = handle.get_variables().get_current_position();
    std::cout << "Current position is " << position << ".\n";
  }

  void TicCtrlr::display_settings()
  {
    std::cout << "TIC SETTINGS: " << handle.get_settings().to_string() << std::endl;
  }

  void TicCtrlr::import_settings()
  {
    std::ifstream ifs("/home/michaelrencheck/FinalProject/src/table_motor_control/config/tic_settings.txt");
    std::string content((std::istreambuf_iterator<char>(ifs) ), (std::istreambuf_iterator<char>() ));

    if (ifs.fail())
    {
      std::cout << "TIC SETTINGS: Failed" << std::endl;
    }

    std::cout << "TIC SETTINGS: \n" << content << std::endl;

    tic::settings s = tic::settings::read_from_string(content);

    handle.set_settings(s);
    std::cout << "TIC SETTINGS imported successfully." << std::endl;
  }

 tic::handle TicCtrlr::open_handle(std::string desired_serial_number)
 {
   // Get a list of Tic devices connected via USB.
   std::vector<tic::device> list = tic::list_connected_devices();

   // Iterate through the list and select one device.
   for (const tic::device & device : list)
   {
     // skip over mismatches
     if(device.get_serial_number() != desired_serial_number) continue;

     // Open a handle to this device and return it.
     return tic::handle(device);
   }

   throw std::runtime_error("No device found with serial number: " + desired_serial_number);
 }


}

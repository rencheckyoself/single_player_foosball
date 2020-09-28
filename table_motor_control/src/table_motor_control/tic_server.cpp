/// \file
/// \brief a wrapper class to interact with Tic C++ API and manage service callbacks

#include <iostream>
#include <fstream>
#include <tic.hpp>

#include "tic_server.hpp"

namespace tic_server
{
  TicCtrlr::TicCtrlr(std::string desired_serial_number, std::string name)
  {
    handle = TicCtrlr::open_handle(desired_serial_number);
    serial_number = desired_serial_number;
    nickname = name;
  }

  void TicCtrlr::OfferServices(ros::NodeHandle &n)
  {
    service = n.advertiseService(nickname + "_update_settings", &TicCtrlr::import_settings, this);
  }

  void TicCtrlr::get_current_pos()
  {
    int32_t position = handle.get_variables().get_current_position();
    std::cout << "Current position is " << position << ".\n";
  }

  void TicCtrlr::set_position(int32_t val)
  {
    handle.halt_and_hold();
    handle.set_target_position(val);
  }

  void TicCtrlr::display_settings()
  {
    std::cout << "TIC SETTINGS: " << handle.get_settings().to_string() << std::endl;
  }

  bool TicCtrlr::import_settings(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    // get file and convert it to a string
    std::ifstream ifs("/home/michaelrencheck/FinalProject/src/table_motor_control/config/tic_settings.txt");
    std::string content((std::istreambuf_iterator<char>(ifs) ), (std::istreambuf_iterator<char>() ));

    // Check if the file was not found
    if (ifs.fail())
    {
      std::cout << "Tic Settings file was not found." << std::endl;
    }

    tic::settings s = tic::settings::read_from_string(content);

    handle.set_settings(s);

    return 1;
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

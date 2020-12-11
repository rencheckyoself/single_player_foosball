#include "tic_server.hpp"

namespace tic_server
{
  TicCtrlr::TicCtrlr(std::string desired_serial_number, std::string name)
  {
    handle = TicCtrlr::open_handle(desired_serial_number); // Initialize the controller
    serial_number = desired_serial_number;
    nickname = name;
    settings_service = n.advertiseService(nickname + "_update_settings", &TicCtrlr::import_settings, this);
  }

  void TicCtrlr::offer_services()
  {
    velocity_service = n.advertiseService(nickname + "_set_target_velocity", &TicCtrlr::set_velocity, this);
    target_service = n.advertiseService(nickname + "_set_target_pos", &TicCtrlr::set_position, this);
    reset_home_service = n.advertiseService(nickname + "_reset_current_pos", &TicCtrlr::reset_global_position, this);
    resume_service = n.advertiseService(nickname + "_resume", &TicCtrlr::resume, this);
    deenergize_service = n.advertiseService(nickname + "_deenergize", &TicCtrlr::deenergize, this);
  }

  int32_t TicCtrlr::get_current_pos()
  {
    return handle.get_variables().get_current_position();
  }

  int32_t TicCtrlr::get_max_speed()
  {
    return handle.get_variables().get_max_speed();
  }

  bool TicCtrlr::set_position(table_motor_control::Int32::Request &req, table_motor_control::Int32::Response &)
  {
    set_position(req.data);
    return 1;
  }

  void TicCtrlr::set_position(int32_t val)
  {
    handle.set_target_position(val);
  }

  bool TicCtrlr::set_velocity(table_motor_control::Int32::Request &req, table_motor_control::Int32::Response &)
  {
    set_position(req.data);
    return 1;
  }

  void TicCtrlr::set_velocity(int32_t val)
  {
    handle.set_target_velocity(val);
  }

  bool TicCtrlr::reset_global_position(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    reset_global_position();
    return 1;
  }

  void TicCtrlr::reset_global_position()
  {
    handle.halt_and_set_position(0);
  }

  void TicCtrlr::halt()
  {
    handle.halt_and_hold();
  }

  void TicCtrlr::display_settings()
  {
    std::cout << "TIC SETTINGS: " << handle.get_settings().to_string() << std::endl;
  }

  void TicCtrlr::import_settings()
  {
    // get file and convert it to a string
    std::string package_path;
    package_path = ros::package::getPath("table_vision_sensing");
    std::ifstream ifs(package_path + "/config/" + nickname + "_settings.txt");
    std::string content((std::istreambuf_iterator<char>(ifs) ), (std::istreambuf_iterator<char>() ));

    // Check if the file was not found
    if (ifs.fail())
    {
      std::cout << "Tic Settings file was not found." << std::endl;
    }
    else
    {
      tic::settings s = tic::settings::read_from_string(content);
      handle.set_settings(s);
    }
  }

  bool TicCtrlr::import_settings(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    import_settings();
    return 1;
  }

  bool TicCtrlr::resume(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    resume();
    return 1;
  }

  void TicCtrlr::resume()
  {
    // set the target to the current position to prevent movement if the motor is not at its target position.
    handle.set_target_position(handle.get_variables().get_current_position());

    handle.exit_safe_start();
    handle.energize();
  }

  bool TicCtrlr::deenergize(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
  {
    deenergize();
    return 1;
  }

  void TicCtrlr::deenergize()
  {
    handle.deenergize();
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

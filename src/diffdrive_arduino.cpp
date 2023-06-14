#include "diffdrive_arduino/diffdrive_arduino.h"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

DiffDriveArduino::DiffDriveArduino()
    : logger_(rclcpp::get_logger("DiffDriveArduino"))
{}

return_type DiffDriveArduino::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

  time_ = chrono::system_clock::now();

  // Get the wheel names
  wheel_name_left_ = info_.hardware_parameters["left_wheel_name"];
  wheel_name_right_ = info_.hardware_parameters["right_wheel_name"];

  // Config
  device_name_  = info_.hardware_parameters["device"];
  baudrate_     = stoi(info_.hardware_parameters["baud_rate"]);
  timeout_      = stoi(info_.hardware_parameters["timeout"]);
  update_rate_  = stoi(info_.hardware_parameters["loop_rate"]);
  encoder_rate_ = stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Init dynamixel
  shared_ptr<dynamixel::PortHandler> port_handler(dynamixel::PortHandler::getPortHandler(device_name_.c_str()));
  shared_ptr<dynamixel::PacketHandler> packet_handler(dynamixel::PacketHandler::getPacketHandler(2.0));

  wheel_left_.setup(1, VELOCITY_CONTROL, port_handler, packet_handler);
  wheel_right_.setup(6, VELOCITY_CONTROL, port_handler, packet_handler);

  RCLCPP_INFO(logger_, wheel_left_.init());
  RCLCPP_INFO(logger_, wheel_right_.init());

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffDriveArduino::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_name_left_, hardware_interface::HW_IF_VELOCITY, &wheel_left_vel_read_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_name_left_, hardware_interface::HW_IF_POSITION, &wheel_left_pos_read_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_name_right_, hardware_interface::HW_IF_VELOCITY, &wheel_right_vel_read_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_name_right_, hardware_interface::HW_IF_POSITION, &wheel_right_pos_read_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduino::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_name_left_, hardware_interface::HW_IF_VELOCITY, &wheel_left_vel_goal_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_name_right_, hardware_interface::HW_IF_VELOCITY, &wheel_right_vel_goal_));

  return command_interfaces;
}


return_type DiffDriveArduino::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  RCLCPP_INFO(logger_, wheel_left_.activate());
  RCLCPP_INFO(logger_, wheel_right_.activate());

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type DiffDriveArduino::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");

  RCLCPP_INFO(logger_, wheel_left_.deactivate());
  RCLCPP_INFO(logger_, wheel_right_.deactivate());

  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::read()
{

  // // TODO fix chrono duration

  // // Calculate time delta
  // auto new_time = std::chrono::system_clock::now();
  // std::chrono::duration<double> diff = new_time - time_;
  // double deltaSeconds = diff.count();
  // time_ = new_time;


  // if (!arduino_.connected())
  // {
  //   return return_type::ERROR;
  // }

  // arduino_.readEncoderValues(l_wheel_.enc, r_wheel_.enc);

  // double pos_prev = l_wheel_.pos;
  // l_wheel_.pos = l_wheel_.calcEncAngle();
  // l_wheel_.vel = (l_wheel_.pos - pos_prev) / deltaSeconds;

  // pos_prev = r_wheel_.pos;
  // r_wheel_.pos = r_wheel_.calcEncAngle();
  // r_wheel_.vel = (r_wheel_.pos - pos_prev) / deltaSeconds;

  return return_type::OK;
}

hardware_interface::return_type DiffDriveArduino::write()
{

  // if (!arduino_.connected())
  // {
  //   return return_type::ERROR;
  // }

  RCLCPP_INFO(logger_, wheel_left_.setVelRPM(wheel_left_vel_goal_));
  RCLCPP_INFO(logger_, wheel_right_.setVelRPM(wheel_right_vel_goal_));
  
  return return_type::OK;  
}



#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffDriveArduino,
  hardware_interface::SystemInterface
)
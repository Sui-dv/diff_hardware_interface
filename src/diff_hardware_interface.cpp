#include "diff_hardware_interface/diff_hardware_interface.h"


DiffHardwareInterface::DiffHardwareInterface()
    : logger_(rclcpp::get_logger("DiffHardwareInterface"))
{}

return_type DiffHardwareInterface::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");

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

std::vector<hardware_interface::StateInterface> DiffHardwareInterface::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_name_left_, hardware_interface::HW_IF_VELOCITY, &wheel_left_vel_read_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_name_left_, hardware_interface::HW_IF_POSITION, &wheel_left_pos_read_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_name_right_, hardware_interface::HW_IF_VELOCITY, &wheel_right_vel_read_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(wheel_name_right_, hardware_interface::HW_IF_POSITION, &wheel_right_pos_read_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffHardwareInterface::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_name_left_, hardware_interface::HW_IF_VELOCITY, &wheel_left_vel_goal_));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(wheel_name_right_, hardware_interface::HW_IF_VELOCITY, &wheel_right_vel_goal_));

  return command_interfaces;
}


return_type DiffHardwareInterface::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  RCLCPP_INFO(logger_, wheel_left_.activate());
  RCLCPP_INFO(logger_, wheel_right_.activate());

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type DiffHardwareInterface::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");

  RCLCPP_INFO(logger_, wheel_left_.deactivate());
  RCLCPP_INFO(logger_, wheel_right_.deactivate());

  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffHardwareInterface::read()
{
  wheel_left_pos_read_ = wheel_left_.getPosDegree();
  wheel_right_pos_read_ = wheel_right_.getPosDegree();
  
  wheel_left_vel_read_ = wheel_left_.getVelRPM();
  wheel_right_vel_read_ = wheel_right_.getVelRPM();

  return return_type::OK;
}

hardware_interface::return_type DiffHardwareInterface::write()
{
  wheel_left_.setVelRPM(wheel_left_vel_goal_);
  wheel_right_.setVelRPM(wheel_right_vel_goal_);
  
  return return_type::OK;  
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffHardwareInterface,
  hardware_interface::SystemInterface
)
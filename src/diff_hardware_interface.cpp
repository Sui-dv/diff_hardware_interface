#include "diff_hardware_interface/diff_hardware_interface.h"


DiffHardwareInterface::DiffHardwareInterface()
    : logger_(rclcpp::get_logger("DiffHardwareInterface"))
{
}

DiffHardwareInterface::~DiffHardwareInterface()
{
  // Deactivate all dynamixels
}

return_type DiffHardwareInterface::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");
  
  // Configure wheel params
  wheel_count_ = stoi(info_.hardware_parameters["wheel_count"]);
  for (auto itr = 1; itr <= wheel_count_; itr++){
    WheelConfig configurator;

    configurator.wheel_name    = info_.hardware_parameters["wheel_name_" + to_string(itr)];
    configurator.wheel_id      = stoi(info_.hardware_parameters["wheel_id_" + to_string(itr)]);
    configurator.mode          = stoi(info_.hardware_parameters["wheel_mode_" + to_string(itr)]);
    configurator.real_hardware = stoi(info_.hardware_parameters["wheel_sim_" + to_string(itr)]);
    configurator.fake_motor    = make_shared<FakeDynamixelHandle>();
    configurator.motor         = make_shared<DynamixelHandle>();

    wheels_.push_back(configurator);
  }

  // Configure port params
  device_name_  = info_.hardware_parameters["device"];
  baudrate_     = stoi(info_.hardware_parameters["baud_rate"]);
  timeout_      = stoi(info_.hardware_parameters["timeout"]);
  update_rate_  = stoi(info_.hardware_parameters["loop_rate"]);
  encoder_rate_ = stoi(info_.hardware_parameters["enc_counts_per_rev"]);

  // Init dynamixel
  shared_ptr<dynamixel::PortHandler> port_handler(dynamixel::PortHandler::getPortHandler(device_name_.c_str()));
  shared_ptr<dynamixel::PacketHandler> packet_handler(dynamixel::PacketHandler::getPacketHandler(2.0));
  
  // Complete wheel setup
  for (auto itr : wheels_){
    itr.fake_motor.get()->setup(itr.wheel_id, itr.mode ? POSITION_CONTROL : VELOCITY_CONTROL , port_handler, packet_handler);
    RCLCPP_INFO(logger_, itr.fake_motor.get()->init(0));
  }

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffHardwareInterface::export_state_interfaces()
{
  // We need to set up a position and a velocity interface for each wheel

  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto itr : wheels_){
    state_interfaces.emplace_back(hardware_interface::StateInterface(itr.wheel_name, hardware_interface::HW_IF_VELOCITY, &itr.encoder_vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(itr.wheel_name, hardware_interface::HW_IF_POSITION, &itr.encoder_pos));
  }

  // state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_[0].wheel_name, hardware_interface::HW_IF_VELOCITY, &wheels_[0].encoder_vel));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_[0].wheel_name, hardware_interface::HW_IF_POSITION, &wheels_[0].encoder_pos));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_[1].wheel_name, hardware_interface::HW_IF_VELOCITY, &wheels_[1].encoder_vel));
  // state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_[1].wheel_name, hardware_interface::HW_IF_POSITION, &wheels_[1].encoder_pos));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffHardwareInterface::export_command_interfaces()
{
  // We need to set up a velocity command interface for each wheel

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto itr : wheels_){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(itr.wheel_name, itr.mode ? hardware_interface::HW_IF_POSITION : hardware_interface::HW_IF_VELOCITY, &itr.goal));
  }

  // command_interfaces.emplace_back(hardware_interface::CommandInterface(wheels_[0].wheel_name, hardware_interface::HW_IF_VELOCITY, &wheels_[0].goal));
  // command_interfaces.emplace_back(hardware_interface::CommandInterface(wheels_[1].wheel_name, hardware_interface::HW_IF_VELOCITY, &wheels_[1].goal));

  return command_interfaces;
}


return_type DiffHardwareInterface::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  RCLCPP_INFO(logger_, wheels_[0].fake_motor.get()->activate());
  RCLCPP_INFO(logger_, wheels_[1].fake_motor.get()->activate());

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type DiffHardwareInterface::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");

  RCLCPP_INFO(logger_, wheels_[0].fake_motor.get()->deactivate());
  RCLCPP_INFO(logger_, wheels_[1].fake_motor.get()->deactivate());

  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type DiffHardwareInterface::read()
{
  wheels_[0].encoder_pos = wheels_[0].fake_motor.get()->getPosDegree();
  wheels_[1].encoder_pos = wheels_[1].fake_motor.get()->getPosDegree();
  
  wheels_[0].encoder_vel = wheels_[0].fake_motor.get()->getVelRPM();
  wheels_[1].encoder_vel = wheels_[1].fake_motor.get()->getVelRPM();

  return return_type::OK;
}

hardware_interface::return_type DiffHardwareInterface::write()
{
  wheels_[0].fake_motor.get()->setVelRPM(wheels_[0].goal);
  wheels_[1].fake_motor.get()->setVelRPM(wheels_[1].goal);
  
  return return_type::OK;  
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  DiffHardwareInterface,
  hardware_interface::SystemInterface
)
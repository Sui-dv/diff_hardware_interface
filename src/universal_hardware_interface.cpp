#include "universal_hardware_interface/universal_hardware_interface.h"


UnivHardwareInterface::UnivHardwareInterface()
    : logger_(rclcpp::get_logger("UnivHardwareInterface"))
{
}

UnivHardwareInterface::~UnivHardwareInterface()
{
  // Deactivate all dynamixels
  for (auto itr : wheels_){
    if (itr.real_hardware){
      RCLCPP_INFO(logger_, itr.real_motor.get()->deactivate());
    } else {
      RCLCPP_INFO(logger_, itr.fake_motor.get()->deactivate());
    }
  }
}

return_type UnivHardwareInterface::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  RCLCPP_INFO(logger_, "Configuring...");
  
  // Configure wheel params
  wheel_count_ = stoi(info_.hardware_parameters["wheel_count"]);
  for (int itr = 1; itr <= wheel_count_; itr++){
    WheelConfig configurator;

    configurator.wheel_name    = info_.hardware_parameters["wheel_name_" + to_string(itr)];
    configurator.wheel_id      = stoi(info_.hardware_parameters["wheel_id_" + to_string(itr)]);
    configurator.real_hardware = stoi(info_.hardware_parameters["real_hardware_" + to_string(itr)]);
    configurator.fake_motor    = make_shared<FakeDynamixelHandle>();
    configurator.real_motor    = make_shared<DynamixelHandle>();

    if (info_.hardware_parameters["wheel_mode_" + to_string(itr)] == "position"){
      configurator.mode = 1;
    } else if (info_.hardware_parameters["wheel_mode_" + to_string(itr)] == "velocity"){
      configurator.mode = 0;
    } else {
      RCLCPP_ERROR(logger_, "Invalid wheel mode, Wheel ID:[%i]", configurator.wheel_id);
      return return_type::ERROR;
    }

    configurator.pos_multiplier  = stod(info_.hardware_parameters["pos_multiplier_" + to_string(itr)]);
    configurator.vel_multiplier  = stod(info_.hardware_parameters["vel_multiplier_" + to_string(itr)]);

    wheels_.push_back(configurator);
  }

  // Configure port params
  device_name_  = info_.hardware_parameters["device"];
  baudrate_     = stoi(info_.hardware_parameters["baud_rate"]);
  update_rate_  = stoi(info_.hardware_parameters["loop_rate"]);

  // Init dynamixel
  shared_ptr<dynamixel::PortHandler> port_handler(dynamixel::PortHandler::getPortHandler(device_name_.c_str()));
  shared_ptr<dynamixel::PacketHandler> packet_handler(dynamixel::PacketHandler::getPacketHandler(2.0));
  
  // Complete wheel setup
  for (auto itr : wheels_){
    if (itr.real_hardware){
      itr.real_motor.get()->setup(itr.wheel_id, itr.mode ? POSITION_CONTROL : VELOCITY_CONTROL , port_handler, packet_handler);
      RCLCPP_INFO(logger_, itr.real_motor.get()->init());
    } else {
      itr.fake_motor.get()->setup(itr.wheel_id, itr.mode ? POSITION_CONTROL : VELOCITY_CONTROL , port_handler, packet_handler);
      RCLCPP_INFO(logger_, itr.fake_motor.get()->init(0));
    }
  }

  RCLCPP_INFO(logger_, "Finished Configuration");

  status_ = hardware_interface::status::CONFIGURED;
  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> UnivHardwareInterface::export_state_interfaces()
{

  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (int itr = 0; itr < wheel_count_; itr++){
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_[itr].wheel_name, hardware_interface::HW_IF_VELOCITY, &wheels_[itr].encoder_vel));
    state_interfaces.emplace_back(hardware_interface::StateInterface(wheels_[itr].wheel_name, hardware_interface::HW_IF_POSITION, &wheels_[itr].encoder_pos));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> UnivHardwareInterface::export_command_interfaces()
{

  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (int itr = 0; itr < wheel_count_; itr++){
    command_interfaces.emplace_back(hardware_interface::CommandInterface(wheels_[itr].wheel_name, wheels_[itr].mode ? hardware_interface::HW_IF_POSITION : hardware_interface::HW_IF_VELOCITY, &wheels_[itr].goal));
  }

  return command_interfaces;
}


return_type UnivHardwareInterface::start()
{
  RCLCPP_INFO(logger_, "Starting Controller...");

  for (auto itr : wheels_){
    if (itr.real_hardware){
      RCLCPP_INFO(logger_, itr.real_motor.get()->activate());
    } else {
      RCLCPP_INFO(logger_, itr.fake_motor.get()->activate());
    }
  }

  status_ = hardware_interface::status::STARTED;

  return return_type::OK;
}

return_type UnivHardwareInterface::stop()
{
  RCLCPP_INFO(logger_, "Stopping Controller...");

  for (auto itr : wheels_){
    if (itr.real_hardware){
      RCLCPP_INFO(logger_, itr.real_motor.get()->deactivate());
    } else {
      RCLCPP_INFO(logger_, itr.fake_motor.get()->deactivate());
    }
  }

  status_ = hardware_interface::status::STOPPED;

  return return_type::OK;
}

hardware_interface::return_type UnivHardwareInterface::read()
{
  for (int itr = 0; itr < wheel_count_; itr++){
    if (wheels_[itr].real_hardware){
      wheels_[itr].encoder_pos = wheels_[itr].real_motor.get()->getPosDegree() * 2*3.14 / 360;     // -> [rad]
      wheels_[itr].encoder_vel = wheels_[itr].real_motor.get()->getVelRPM();                       // -> [rpm]
    } else {
      wheels_[itr].encoder_pos = wheels_[itr].fake_motor.get()->getPosDegree() * 2*3.14 / 360;
      wheels_[itr].encoder_vel = wheels_[itr].fake_motor.get()->getVelRPM();

      wheels_[itr].fake_motor.get()->simStep(1);
    }
  }

  return return_type::OK;
}

hardware_interface::return_type UnivHardwareInterface::write()
{
  for (auto itr : wheels_){
    if (itr.real_hardware){
      if (itr.mode){
        itr.real_motor.get()->setPosDegree(itr.goal * itr.pos_multiplier* 360 /2/3.14);
      } else {
        itr.real_motor.get()->setVelRPM(itr.goal * itr.vel_multiplier);
      }
    } else {
      if (itr.mode){
        itr.fake_motor.get()->setPosDegree(itr.goal * itr.pos_multiplier * 360 /2/3.14);
      } else {
        itr.fake_motor.get()->setVelRPM(itr.goal * itr.vel_multiplier);
      }
    }
  }

  return return_type::OK;  
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  UnivHardwareInterface,
  hardware_interface::SystemInterface
)
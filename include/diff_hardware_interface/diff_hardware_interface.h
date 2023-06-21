#ifndef DIFF_HARDWARE_INTERFACE_H
#define DIFF_HARDWARE_INTERFACE_H

#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "dynamixel_wrapper/wheel_config.hpp"

using hardware_interface::return_type;
using namespace std;

class DiffHardwareInterface : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{


public:
  DiffHardwareInterface();

  ~DiffHardwareInterface();

  return_type configure(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type start() override;

  return_type stop() override;

  return_type read() override;

  return_type write() override;

private:
  vector<WheelConfig>     wheels_;
  uint16_t                wheel_count_;

  string                  wheel_name_left_;
  string                  wheel_name_right_;

  string                  device_name_;     // Serial port
  uint32_t                baudrate_;        // Baudrate
  uint16_t                timeout_;         // Serial timeout
  uint16_t                update_rate_;     // Write rate
  uint16_t                encoder_rate_;    // Read rate

  rclcpp::Logger          logger_;  
};


#endif // DIFF_HARDWARE_INTERFACE_H
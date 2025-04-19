#ifndef MSDS_INTERFACE_HPP // If not already defined
#define MSDS_INTERFACE_HPP // Include guard to prevent multiple inclusions

// The guard prevents the class from being defined multiple times, which can lead to
// redefinition errors. This is a common practice in C++ header files to ensure that
// the class is only defined once, even if the header file is included multiple times
// in different translation units.

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <vector>
#include <string>

// This header file is part of the MSDS firmware interface for ROS 2.
// It defines the MSDSInterface class, which is responsible for communicating
// with the MSDS hardware via a serial port. The class inherits from
// hardware_interface::SystemInterface and implements the necessary methods
// to read and write data to the hardware. It also handles the lifecycle
// management of the node, allowing it to be activated and deactivated
// properly. The class uses the LibSerial library for serial communication
// and includes the necessary headers for working with ROS 2 lifecycle nodes
// and hardware interfaces.

namespace msds_firmware
{
    // an alias for the CallbackReturn type from the lifecycle node interface
    // This is used to simplify the code and make it more readable
    // CallbackReturn is a type that represents the return value of lifecycle
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class MSDSInterface : public hardware_interface::SystemInterface
    {
    public:
        MSDSInterface(); // Constructor
        virtual ~MSDSInterface(); //

        // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
        // Redefining the lifecycle methods to manage the state of the node
        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        // Implementing hardware_interface::SystemInterface
        CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    private:
        LibSerial::SerialPort arduino_;
        std::string port_;
        std::vector<double> velocity_commands_;
        std::vector<double> position_states_;
        std::vector<double> velocity_states_;
        rclcpp::Time last_run_;
    };
}  // namespace msds_firmware


#endif  // MSDS_INTERFACE_HPP
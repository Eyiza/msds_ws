#ifndef MSDS_INTERFACE_HPP // If not already defined
#define MSDS_INTERFACE_HPP // Include guard to prevent multiple inclusions

// The guard prevents the class from being defined multiple times, which can lead to
// redefinition errors. This is a common practice in C++ header files to ensure that
// the class is only defined once, even if the header file is included multiple times
// in different translation units.

#include <rclcpp/rclcpp.hpp> // Core ROS 2 C++ node features.
#include <hardware_interface/system_interface.hpp> // Base class for hardware interfaces. It makes this class a hardware interface for ros2_control.
#include <libserial/SerialPort.h> // LibSerial library for serial communication.
#include <rclcpp_lifecycle/state.hpp> // Lifecycle state management.
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp> // Lifecycle node interface.
#include <sensor_msgs/msg/joint_state.hpp> // ROS 2 message type for joint states.
#include <vector> // Standard library for using vectors which are dynamic arrays / lists.
#include <string> // Standard library for using strings.

// This header file is part of the MSDS firmware interface for ROS 2.
// It defines the MSDSInterface class, which is responsible for communicating
// with the MSDS hardware via a serial port. The class inherits from
// hardware_interface::SystemInterface and implements the necessary methods
// to read and write data to the hardware. It also handles the lifecycle
// management of the node, allowing it to be activated and deactivated
// properly. The class uses the LibSerial library for serial communication
// and includes the necessary headers for working with ROS 2 lifecycle nodes
// and hardware interfaces.

namespace msds_firmware // Namespace for the MSDS firmware interface
{
    // an alias for the CallbackReturn type from the lifecycle node interface
    // This is used to simplify the code and make it more readable
    // CallbackReturn is a type that represents the return value of lifecycle
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // Create a class named MSDSInterface that inherits from hardware_interface::SystemInterface
    class MSDSInterface : public hardware_interface::SystemInterface
    {
    public:
        MSDSInterface(); // Constructor - runs when the object is created.
        virtual ~MSDSInterface(); // Destructor - runs when the object is deleted.

        // Methods
        // Implementing rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
        // Redefining the lifecycle methods to manage the state of the node
        // virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &) override; // overrides the vitual method
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

        // Implementing hardware_interface::SystemInterface
        CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override; // Called when ROS first loads your hardware.
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override; // Exports the state interfaces of the hardware.
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override; // Exports the command interfaces of the hardware.

        // Logic for reading and writing data to/from the hardware.
        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override; // Reads the state of the hardware.
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override; // Writes commands to the hardware.

    private:
        // Properties
        LibSerial::SerialPort arduino_; // Object to communicate with your Arduino over USB.
        std::string port_; // The port name where the Arduino is connected.

        /* 
            We have four motors (front right, front left, rear right, rear left).
            Recall: To connect ros2 control to the hardware, we need to define the command and state interfaces for each joint.
            The command_interface defines how we want to write messages to the hardware.
            The state_interface defines how we want to read/recieve messages or feedback from the hardware like motors.
            There are 3 control modes: position, velocity, and effort. 
            In this case, we want to control the speed of the motors, so we use velocity command interfaces.
            We also want to read the position and speed of the motors, so we use position and velocity state interfaces.
            The command and state interfaces are stored in vectors (dynamic arrays) to allow for easy resizing.
        */
        std::vector<double> velocity_commands_; // Speed commands for the motors.
        std::vector<double> position_states_; // Current position of the motors.
        std::vector<double> velocity_states_; // Current speed of the motors.

        rclcpp::Time last_run_; // The last time the read/write functions were called.

        // Publisher for joint/wheel states.
        rclcpp::Node::SharedPtr node_;  // Needed to create a publisher
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr wheel_state_pub_; // Publisher for joint/wheel states.
    };
}  // namespace msds_firmware


#endif  // MSDS_INTERFACE_HPP
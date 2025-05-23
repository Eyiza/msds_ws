#include "msds_firmware/msds_interface.hpp" // Header file created previously for the MSDSInterface class
#include <hardware_interface/types/hardware_interface_type_values.hpp> // For hardware interface type values
#include <pluginlib/class_list_macros.hpp> // Lets ROS2 dynamically load this class like a plugin

 
namespace msds_firmware
{

  // Constructor for the MSDSInterface class - Called when the object is first created.
  MSDSInterface::MSDSInterface()
  {
  }

  // Destructor for the MSDSInterface class - Runs when the object is destroyed.
  // It closes the serial connection to arduino if it is open.
  MSDSInterface::~MSDSInterface()
  {
    if (arduino_.IsOpen())
    {
      try
      {
        arduino_.Write("flp00.00,frn00.00,rrp00.00,rln00.00,\r"); // Stop all motors
        arduino_.Close();
      }
      catch (...)
      {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("MSDSInterface"),
                            "Something went wrong while closing connection with port " << port_);
      }
    }
  }

  // This function is called when the hardware interface is initialized - when ROS2 is setting up your hardware.
  CallbackReturn MSDSInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
  {
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    // If error
    if (result != CallbackReturn::SUCCESS)
    {
      return result;
    }

    try
    {
      // info_ is inherited from SystemInterface
      // Note that .hardware_parameters.at() can also be written as .hardware_parameters["port"]
      port_ = info_.hardware_parameters.at("port"); // Get the port name from the hardware parameters
      // RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"), "Port: %s", port_.c_str());
    }
    catch (const std::out_of_range &e)
    {
      RCLCPP_FATAL(rclcpp::get_logger("MSDSInterface"), "No Serial Port provided! Aborting");
      return CallbackReturn::FAILURE;
    }

    // Reserve space for the command and state vectors to match the number of joints.
    // This is done to avoid reallocating memory every time we add a new element
    velocity_commands_.reserve(info_.joints.size());
    position_states_.reserve(info_.joints.size());
    velocity_states_.reserve(info_.joints.size());

    last_run_ = rclcpp::Clock().now();

    // Initialize wheel state publisher
    node_ = std::make_shared<rclcpp::Node>("msds_hardware_node");
    wheel_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("wheel_states", 10);

    return CallbackReturn::SUCCESS;
  }


  // This function returns a vector (list) of state interfaces.
  std::vector<hardware_interface::StateInterface> MSDSInterface::export_state_interfaces()
  {
    // Vector variable to store state_intefaces
    std::vector<hardware_interface::StateInterface> state_interfaces;
    RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"), "Exporting state interfaces ...");

    // Provide only a position Interaface
    for (size_t i = 0; i < info_.joints.size(); i++) // For each joint. Note that size_t is an unsigned integer type used in C++ for array indexing.
    {
      // Add the state interfaces to the vector
      // emplace_back(...) is like Python’s list.append(...), but more efficient. 
      // info_.joints[i].name → name of the joint
      // &position_states_[i] → pointer to the memory where the current joint’s position is stored
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));

      // state_interfaces.emplace_back(hardware_interface::StateInterface(
      //     info_.joints[i].name, "position", &position_states_[i]));
      // state_interfaces.emplace_back(hardware_interface::StateInterface(
      //     info_.joints[i].name, "velocity", &velocity_states_[i]));
      
      // RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"),
      //   "[%s] vel: %f, pos: %f",
      //   info_.joints[i].name.c_str(),
      //   &velocity_states_[i],
      //   &position_states_[i]);
    }

    return state_interfaces;
  } 


  // returns command interfaces that ROS2 will use to send commands to your robot.
  std::vector<hardware_interface::CommandInterface> MSDSInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    // Provide only a velocity Interaface
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
    }

    return command_interfaces;
  }


  // on_activate(...) is part of the lifecycle API of ROS2 hardware components. 
  // It gets called when the controller manager starts the robot.
  CallbackReturn MSDSInterface::on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"), "Starting robot hardware ...");
    
    // Reset commands and states
    velocity_commands_.resize(info_.joints.size(), 0.0);
    position_states_.resize(info_.joints.size(), 0.0);
    velocity_states_.resize(info_.joints.size(), 0.0);
    // OR:
    // velocity_commands_ = { 0.0, 0.0, 0.0, 0.0 };
    // position_states_ = { 0.0, 0.0, 0.0, 0.0 };
    // velocity_states_ = { 0.0, 0.0, 0.0, 0.0 };

    try
    {
      // Opens the serial port and sets the baud rate.
      arduino_.Open(port_);
      arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_500000);
      arduino_.FlushIOBuffers();
      RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"),
                  "Opened port %s with baud rate %d", port_.c_str(), 500000);
    }
    catch (...) // Catches any exception
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("MSDSInterface"),
                          "Something went wrong while interacting with port " << port_);
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"),
                "Hardware started, ready to take commands");
    return CallbackReturn::SUCCESS;
  }

  // Similar to on_activate, but the reverse.
  CallbackReturn MSDSInterface::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"), "Stopping robot hardware ...");

    if (arduino_.IsOpen())
    {
      try
      {
        arduino_.Write("flp00.00,frn00.00,rrp00.00,rln00.00,\r"); // Stop all motors
        arduino_.Close();
      }
      catch (...)
      {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("MSDSInterface"),
                            "Something went wrong while closing connection with port " << port_);
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"), "Hardware stopped");
    return CallbackReturn::SUCCESS;
  }

  // This is where the robot reads sensor data sent by Arduino.
  // ROS2 calls this periodically (e.g., 100Hz) to read data from hardware.
  hardware_interface::return_type MSDSInterface::read(const rclcpp::Time &,
                                                            const rclcpp::Duration &)
  {
    // Interpret the string
    if(arduino_.IsDataAvailable()) // Checks if there's something to read.
    {
      // RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"), "Reading from port %s", port_.c_str());
      // Calculates how much time has passed since last read() 
      // — used for integrating velocity into position.
      auto dt = (rclcpp::Clock().now() - last_run_).seconds(); 

      std::string message; 
      // arduino_.FlushIOBuffers();
      arduino_.ReadLine(message); // Gets a full line from serial e.g "flp0.23,frn0.21,rrp0.25,rln0.22"
      if (message.empty() || message.find(',') == std::string::npos) {
        RCLCPP_WARN(rclcpp::get_logger("MSDSInterface"), "Invalid message: %s", message.c_str());
        return hardware_interface::return_type::OK;
      }

      // RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"),
      //             "Received message: %s", message.c_str()); 

      std::stringstream ss(message); // Creates a string stream from the message.
      std::string res;
      int multiplier = 1;

      size_t index = 0;

      while(std::getline(ss, res, ',') && index < velocity_states_.size()) // Splits the message into parts by comma.
      {
        multiplier = res.at(2) == 'p' ? 1 : -1; // Extracts the sign. res.at(2) accesses the third character ('p' or 'n').

        // Convert the string after sign into a float and stores it as velocity.
        // std::stod() = string to double 
        // substr(3) means “slice from index 3 onward”.
        velocity_states_.at(index) = multiplier * std::stod(res.substr(3, res.size()));

        // Integrate velocity over time to update position.
        position_states_.at(index) += velocity_states_.at(index) * dt;
        
        // RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"),
        //   "Wheel[%d] (%s) velocity = %.3f, position = %.3f",
        //   index,
        //   info_.joints[index].name.c_str(),
        //   velocity_states_.at(index),
        //   position_states_.at(index));

        index++;
      }
      // Publish the wheel states
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = rclcpp::Clock().now();
      msg.name = {"front_left_wheel_joint", "front_right_wheel_joint", "rear_right_wheel_joint", "rear_left_wheel_joint"};
      msg.position = position_states_;
      msg.velocity = velocity_states_;
      wheel_state_pub_->publish(msg);

      // update last_run_
      last_run_ = rclcpp::Clock().now();
    }
    return hardware_interface::return_type::OK;
  }


  // This is where we send commands to the Arduino to move the wheels.
  hardware_interface::return_type MSDSInterface::write(const rclcpp::Time &,
                                                            const rclcpp::Duration &)
  {
    if (!arduino_.IsOpen()) return hardware_interface::return_type::ERROR;
    
    // Implement communication protocol with the Arduino
    std::stringstream message_stream;
    // const std::vector<std::string> wheel_labels = {"fr", "fl", "rr", "rl"};
    const std::vector<std::string> wheel_labels = {"fl", "fr", "rr", "rl"};

    for (size_t i = 0; i < velocity_commands_.size(); ++i) {
      char sign = velocity_commands_.at(i) >= 0 ? 'p' : 'n';
      double speed = std::abs(velocity_commands_.at(i));
      std::string pad_zero = (speed < 10.0) ? "0" : ""; // pad_zero is used to add a leading zero if speed is less than 10.0

      // Prepares the message string to be sent to Arduino
      // std::fixed + std::setprecision(2) makes sure we get two decimal places.
      message_stream << wheel_labels[i] << sign << pad_zero << std::fixed << std::setprecision(2) << speed << ",";
    }
    message_stream << "\r"; 
    try
    {
      arduino_.FlushIOBuffers();
      arduino_.Write(message_stream.str()); // Sends the message string to arduino
      // RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"),
      //             "Sent Message");
      // std::this_thread::sleep_for(std::chrono::milliseconds(900));
      

      // std::string response = "";
      // try
      // {
      //   // Responses end with \r\n so we will read up to (and including) the \n.
      //   arduino_.ReadLine(response, '\n', 0);
      // }
      // catch (const LibSerial::ReadTimeout&)
      // {
      //   RCLCPP_WARN(rclcpp::get_logger("MSDSInterface"),
      //               "Read timeout occurred while waiting for response from Arduino.");
      // }
      // RCLCPP_INFO(rclcpp::get_logger("MSDSInterface"),
      //             "Received response: %s", response.c_str());
      // RCLCPP_INFO_STREAM(rclcpp::get_logger("MSDSInterface"),
      //             "Sent with str: " << message_stream.str() << " Recv: " << response);

    }
    catch (...)
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("MSDSInterface"),
                          "Something went wrong while sending the message "
                              << message_stream.str() << " to the port " << port_);
      return hardware_interface::return_type::ERROR;
    }

    return hardware_interface::return_type::OK;
  }
}  // namespace msds_firmware

// This is a macro that registers your class as a ROS2 hardware plugin.
// Without it, ROS2 won't know how to load your class from the .so library during runtime.
PLUGINLIB_EXPORT_CLASS(msds_firmware::MSDSInterface, hardware_interface::SystemInterface)
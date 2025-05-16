import time
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyTurbo
from rclpy.action import ActionClient
from visualization_msgs.msg import Marker, MarkerArray

'''
    This script implements a safety stop mechanism for a robot using laser scan data.
    It subscribes to a laser scan topic and checks for obstacles within specified distances.
    When an obstacle is detected within the warning distance, it sends a decrease speed command.
    When an obstacle is detected within the danger distance, it sends a safety stop command.
    It also visualizes the warning and danger zones using markers.
'''

class State(Enum):  # Enum to represent the state of the robot
    FREE = 0
    WARNING = 1
    DANGER = 2


class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')

        # Declare parameters to make the node reusable by another application
        self.declare_parameter('warning_distance', 0.6) # Warning threshold. Anything below this distance will trigger a decrease in speed.
        self.declare_parameter('danger_distance', 0.2) # Danger threshold. Anything below this distance will trigger a stop.
        self.declare_parameter('scan_topic', 'scan') # Topic to subscribe to for laser scan data. This is the topic where the laser scan data is published.
        self.declare_parameter('safety_stop_topic', 'safety_stop') # Topic to publish safety stop commands. This indicates the name of the lock topic that the twist mux is using in order to determine whether or not to allow sending velocity messages to the robot.

        # Get parameters
        self.warning_distance = self.get_parameter('warning_distance').get_parameter_value().double_value
        self.danger_distance = self.get_parameter('danger_distance').get_parameter_value().double_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter('safety_stop_topic').get_parameter_value().string_value
        
        self.is_first_msg = True
        self.state = State.FREE
        self.prev_state = State.FREE

        # Subscribe to the laser scan topic 
        self.laser_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.laser_callback, 10
        )

        # Create a publisher for the safety stop topic
        self.safety_stop_pub = self.create_publisher(
            Bool, self.safety_stop_topic, 10
        )

        # Create a publisher for the zones topic
        # This topic is used to visualize the warning and danger zones in RViz
        # The zones are represented as markers in RViz        
        self.zones_pub = self.create_publisher(
            MarkerArray, 'zones', 10
        )

        # Create action clients for decreasing and increasing speed
        # These action clients would send goals to the twist mux action server
        self.decrease_speed_client = ActionClient(self, JoyTurbo, 'joy_turbo_decrease')
        self.increase_speed_client = ActionClient(self, JoyTurbo, 'joy_turbo_increase')

        # Wait for the action servers to be available
        while not self.decrease_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn('Action /joy_turbo_decrease not available! Waiting..')
            time.sleep(2.0)
        while not self.increase_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn('Action /joy_turbo_increase not available! Waiting..')
            time.sleep(2.0)

        # Prepare Zones for Visualization
        self.zones = MarkerArray()

        # Warning Zone
        warning_zone = Marker()
        warning_zone.id = 0 # ID of the marker. This is used to identify the marker in RViz
        warning_zone.type = Marker.CYLINDER # This marker is a cylinder
        warning_zone.action = Marker.ADD # This marker is added to the visualization
        warning_zone.scale.z = 0.001 # Height of the cylinder
        warning_zone.scale.x = self.warning_distance * 2 # Diameter of the cylinder which is the same as 2 * the warning distance radius
        warning_zone.scale.y = self.warning_distance * 2
        warning_zone.color.r = 1.0 # Red color
        warning_zone.color.g = 0.984 # Green color
        warning_zone.color.b = 0.0 # Yellow color
        warning_zone.color.a = 0.5 # Transparency of the marker

        # Danger Zone
        danger_zone = Marker()
        danger_zone.id = 1
        danger_zone.type = Marker.CYLINDER
        danger_zone.action = Marker.ADD
        danger_zone.scale.z = 0.001
        danger_zone.scale.x = self.danger_distance * 2
        danger_zone.scale.y = self.danger_distance * 2
        danger_zone.color.r = 1.0
        danger_zone.color.g = 0.0
        danger_zone.color.b = 0.0
        danger_zone.color.a = 0.5
        danger_zone.pose.position.z = 0.01 # Slightly above the ground to avoid overlapping
        
        self.zones.markers = [warning_zone, danger_zone]

        self.standoff_angles = []  # Will store detected standoff angles
        self.standoff_update_threshold = 5  # Degrees tolerance for standoff detection
        self.scan_count = 0
        self.standoff_detection_phase = True  # Start in calibration mode

    # Execute when a new laser scan message is received
    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE

        for i, range_value in enumerate(msg.ranges): # msg.ranges is a list of distances to obstacles
            angle = msg.angle_min + i * msg.angle_increment # Calculate the angle of the laser beam

            # Ignore angles where the standoffs are located
            deg = math.degrees(angle)
            if any(abs(deg - d) < 5 for d in [45, 135, 225, 315]):
                continue

            # Check if the range value is valid (not NaN or Inf)
            if not math.isinf(range_value) and range_value <= self.warning_distance:
                # If in the warning zone, set the state to WARNING
                self.state = State.WARNING

                if range_value <= self.danger_distance:
                    self.state = State.DANGER
                    # Stop immediately!
                    break

        if self.state != self.prev_state: # To avoid sending the same command multiple times
            is_safety_stop = Bool()

            if self.state == State.WARNING: # Robot is in warning state
                is_safety_stop.data = False
                self.decrease_speed_client.send_goal_async(JoyTurbo.Goal()) # Decrease speed
                # Set the transparency of the markers to indicate the state. ID 0 is the warning zone and ID 1 is the danger zone
                self.zones.markers[0].color.a = 1.0
                self.zones.markers[1].color.a = 0.5
            elif self.state == State.DANGER: # Robot is in danger
                is_safety_stop.data = True
                self.zones.markers[0].color.a = 1.0 # If an object is the danger zone, it is also in the warning zone
                self.zones.markers[1].color.a = 1.0
            elif self.state == State.FREE: # Robot is free to move
                is_safety_stop.data = False
                self.increase_speed_client.send_goal_async(JoyTurbo.Goal()) # Increase speed
                # Reset the transparency of the markers to indicate the state
                self.zones.markers[0].color.a = 0.5
                self.zones.markers[1].color.a = 0.5

            self.prev_state = self.state # Update the previous state
            self.safety_stop_pub.publish(is_safety_stop) # Publish the safety stop command

        if self.is_first_msg:
            # Set the frame ID for the markers to the frame ID of the laser scan message
            # So the markers moves as the robot moves
            for zone in self.zones.markers:
                zone.header.frame_id = msg.header.frame_id

            self.is_first_msg = False # Update variable

        self.zones_pub.publish(self.zones) # Publish Marker zones


def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

 

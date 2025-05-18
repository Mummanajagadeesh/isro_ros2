import rclpy
from rclpy.node import Node
import sys
import signal
import time

from geometry_msgs.msg import Pose
from std_msgs.msg import String

from pymavlink import mavutil

class DroneContrller(Node):
    def __init__(self):
        super().__init__('drone_controller')

        self.pose_subscription = self.create_subscription(
            Pose,
            'orbslam_pose',
            self.pose_callback,
            10
        )
        self.pose_subscription

        self.set_mode_subscription = self.create_subscription(
            String,
            'set_mode',
            self.set_mode,
            10
        )

        self.connection_string = sys.argv[1]
        self.baud_rate = sys.argv[2]

        self.master = self.connect_to_pixhawk()
        self.disable_safety_switch()
        # self.arm_drone()

        self.mode = self.get_mode()

    def connect_to_pixhawk(self):
        print("Connecting to Pixhawk...")
        master = mavutil.mavlink_connection(self.connection_string, baud=self.baud_rate)
        master.wait_heartbeat()
        print("Heartbeat received! Pixhawk is connected.")
        return master

    def disable_safety_switch(self):
        print("Disabling safety switch...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER,
            0, 220, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)
        print("Safety switch disabled.")

    def arm_drone(self):
        print("Arming drone...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Motors armed.")

    def get_mode(self):
        try:
            mode = self.master.flightmode
            print(f"CURRENT MODE: {mode}")
            return mode
        except Exception as e:
            print(f"Error getting mode: {e}")
            return None

    def set_mode(self, new_mode_msg):
        new_mode = new_mode_msg.data
        print(f"Trying to set mode to {new_mode}")
        mode_id = self.master.mode_mapping()[new_mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"Mode set to {new_mode}")
        time.sleep(2)
    
    def pose_callback(self, msg):
        self.get_logger().info(f"RECEIVED: \nPosition = {msg.position.x}, {msg.position.y}, {msg.position.y} \nOrientation = {msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}\n")

        timestamp = int(time.time() * 1e6)
        self.master.mav.vision_position_estimate_send(
            timestamp,
            msg.position.x, msg.position.y, msg.position.z,
            0.0, 0.0, 0.0
        )
        time.sleep(0.1)  # 10 Hz update



def main(args=None):
    if len(sys.argv) < 3:
        print("USAGE: ros2 run drone_control_pkg drone_controller <connection_string> <baud_rate>")
        return

    rclpy.init(args=args)
    drone_controller = DroneContrller()
    rclpy.spin(drone_controller)

    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

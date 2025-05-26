import rclpy
from rclpy.node import Node
import sys
import signal
import time
import threading

from geometry_msgs.msg import Pose
from std_msgs.msg import String

from pymavlink import mavutil

from pynput import keyboard

class DroneController(Node):
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

        #self.imu_publisher = self.create_publisher()

        # To detect 'L' key to land
        self.keyboard_listener = keyboard.Listener(on_press=self.on_keyboard_press)
        self.keyboard_listener.start()

        self.connection_string = sys.argv[1]
        self.baud_rate = sys.argv[2]

        self.master = self.connect_to_pixhawk()
        
        pixhawk_boot_time = self.get_pixhawk_boot_time()
        self.boot_sys_time = time.time()


        self.request_imu_data()

        self.imu_data_thread = threading.Thread(target=self.get_imu_data)
        self.imu_data_thread.daemon = True
        self.imu_data_thread.start()


        self.disable_safety_switch()
        # self.arm_drone()


        self.mode = self.get_mode()

    def connect_to_pixhawk(self):
        print("Connecting to Pixhawk...")
        master = mavutil.mavlink_connection(self.connection_string, baud=self.baud_rate)
        master.wait_heartbeat()
        print("Heartbeat received! Pixhawk is connected.")
        return master

    def get_pixhawk_boot_time(self):
        print("Getting system time for timestamping.")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME,
            1000000,
            0,0,0,0,0
        )

        timeout = 10 # timeout to wait for msg to come
        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.master.recv_match(type="SYSTEM_TIME", blocking=False)
            if msg:
                print(f"Received system time: {msg.time_unix_usec}")
                print(f"Received boot time: {msg.time_boot_ms}")
                return msg

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

    def on_keyboard_press(self, key):
        try:
            if key.char.lower() == 'l':
                self.get_logger().info("L KEY PRESSED: Initiated Landing.")
                land_msg = String()
                land_msg.data = "LAND"
                self.set_mode(land_msg)
        except AttributeError:
            pass

    def request_imu_data(self):
        # SCALED_IMU (Message ID 26)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            26,  # SCALED_IMU message ID
            100000,  # 10Hz = 100000 microseconds
            0, 0, 0, 0, 0
        )
        
        # RAW_IMU (Message ID 27)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            27,  # RAW_IMU message ID
            100000,  # 10Hz
            0, 0, 0, 0, 0
        )    


    def get_imu_data(self):
        # RAW_IMU, SCALED_IMU, SCALED_IMU2, SCALED_IMU3, HIGHRES_IMU
        imu_messages = ['SCALED_IMU']
        while rclpy.ok():
            msg = self.master.recv_match(type=imu_messages,blocking=False)
            if msg:
                imu_data = {}
                imu_data["time_boot_ms"] = msg.time_boot_ms
                imu_data["acc"] = [msg.xacc, msg.yacc, msg.zacc]
                imu_data["gyro"] = [msg.xgyro, msg.ygyro, msg.zgyro]
                self.send_imu_data(imu_data)

    def send_imu_data(self, imu_data):
        print(imu_data)
        pass
        # TODO: Define custon msg interface for imu data and publish it        


    def __del__(self):
        if hasattr(self, 'keyboard_listener'):
            self.keyboard_listener.stop()



def main(args=None):
    if len(sys.argv) < 3:
        print("USAGE: ros2 run drone_control_pkg drone_controller <connection_string> <baud_rate>")
        return

    rclpy.init(args=args)
    drone_controller = DroneController()
    rclpy.spin(drone_controller)

    drone_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

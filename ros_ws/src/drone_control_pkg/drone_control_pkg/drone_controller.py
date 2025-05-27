import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from collections import deque

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # initialization 
        self.connected = False
        self.armed = False
        self.mode = ""
        self.arm_future = None
        self.disarm_future = None
        self.set_mode_future = None
        self.slam_tracking_state = False

        self.pose_buffer = deque(maxlen=10)

        # to check futures
        self.check_futures_timer = self.create_timer(0.1, self.check_futures)
        self.publish_pose_timer = self.create_timer(0.1, self.vision_pose_cb)

        # subscribe to the manual drone controller (use for testing)
        self.manual_drone_cmds_subscription = self.create_subscription(String, "/manual_drone_cmds", self.manual_drone_cmd_cb, 10)

        # get the current state of the drone
        self.state_subscription = self.create_subscription(State, "mavros/state", self.state_cb, 10)

        # service to arm and disarm the drone
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')

        # service to set mode
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')

        # subscribe to the ORB_SLAM3 node
        self.slam_subscription = self.create_subscription(PoseStamped, "/orbslam_pose", self.slam_cb, 10)

        self.vision_pose_estimate_publisher = self.create_publisher(PoseStamped, "mavros/vision_pose/pose", 10)

    # callback for manual_drone_controller
    def manual_drone_cmd_cb(self, msg):
        self.get_logger().info(f"RECEIVED COMMAND: {msg.data}")
        if msg.data == "arm":
            self.arm_drone()
        elif msg.data == "disarm":
            self.disarm_drone()
        elif msg.data.startswith("mode"):
            self.set_mode(msg.data.split()[1])
        elif msg.data == "land" or msg.data == "l":
            self.set_mode("LAND")

    # callback for state topic 
    def state_cb(self, msg):
        print("***")
        print(f"CONNECTED: {msg.connected}")
        print(f"ARMED: {msg.armed}")
        print(f"MODE: {msg.mode}")
        print("***")
        self.mode = msg.mode
        self.connected = msg.connected
        self.armed = msg.armed

    # calls the arming service to arm drone
    def arm_drone(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ERROR: arming service not available.')
            return

        self.arming_req = CommandBool.Request()
        self.arming_req.value = True

        # Arming the drone
        self.arm_future = self.arming_client.call_async(self.arming_req)


    # calls the arming service to disarm drone
    def disarm_drone(self):
        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ERROR: arming service not available.')
            return

        self.disarming_req = CommandBool.Request()
        self.disarming_req.value = False

        # Disarming the drone
        self.disarm_future = self.arming_client.call_async(self.disarming_req)

    # set the mode 
    def set_mode(self, mode):
        while not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('ERROR: set_mode service not available.')
            return
        
        self.mode_req = SetMode.Request()
        self.mode_req.custom_mode = mode.upper()

        self.set_mode_future = self.mode_client.call_async(self.mode_req)


    def slam_cb(self, msg):
        pos_x = msg.pose.position.x
        pos_y = msg.pose.position.y
        pos_z = msg.pose.position.z
        q_x = msg.pose.orientation.x
        q_y = msg.pose.orientation.y
        q_z = msg.pose.orientation.z
        q_w = msg.pose.orientation.w

        # to check if orb slam is not actually tracking now
        if all([pos_x, pos_y, pos_z, q_x, q_y, q_z]) and q_w == 1.000:
            if self.slam_tracking_state:
                print("ORB SLAM STOPPED TRACKING!")
                self.slam_tracking_state = False
            return
        
        print("*** RECEIVED SLAM POSE ***")
        print(f"Timestamp: {msg.header.stamp}")
        print(f"Position: {pos_x} {pos_y} {pos_z}")
        print(f"Orientation: {q_x} {q_y} {q_z} {q_w}")
        print("**************************")

        self.slam_tracking_state = True

        self.pose_buffer.append(msg)

    def vision_pose_cb(self):
        if self.pose_buffer:
            pose = self.pose_buffer.popleft() # earliest pose in the buffer
            self.vision_pose_estimate_publisher.publish(pose)
            self.get_logger().info(f"Published vision pose estimate (timestamp: {pose.header.stamp})")


    def check_futures(self):
        if self.arm_future is not None and self.arm_future.done():
            try:
                result = self.arm_future.result()
                if result.success:
                    self.get_logger().info('Arming command sent successfully')
                else:
                    self.get_logger().error('Arming failed.')
            except Exception as e:
                self.get_logger().error(f'Arming service call failed: {e}')
            finally:
                self.arm_future = None 
            
        if self.disarm_future is not None and self.disarm_future.done():
            try:
                result = self.disarm_future.result()
                if result.success:
                    self.get_logger().info('Disarming command sent successfully')
                else:
                    self.get_logger().error('Disarming failed.')
            except Exception as e:
                self.get_logger().error(f'Disarming service call failed: {e}')
            finally:
                self.disarm_future = None 

        if self.set_mode_future is not None and self.set_mode_future.done():
            try:
                result = self.set_mode_future.result()
                if result.success:
                    self.get_logger().info('Set mode command sent successfully')
                else:
                    self.get_logger().error('Set mode failed.')
            except Exception as e:
                self.get_logger().error(f'Set mode service call failed: {e}')
            finally:
                self.set_mode_future = None 
        

 

def main(args=None):
    rclpy.init(args=args)
    drone_controller = DroneController()

    try:
        rclpy.spin(drone_controller)
    except KeyboardInterrupt:
        drone_controller.get_logger().info('Shutting down...')
    finally:
        drone_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

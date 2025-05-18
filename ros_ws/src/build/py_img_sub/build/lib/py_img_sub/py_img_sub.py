
import rclpy
from rclpy.node import Node

from image_transport_py import ImageTransport

class PyImgSub(Node):
    def __init__(self):
        super().__init__('py_img_sub')

        image_transport = ImageTransport(
            'camera_feed_sub', 
            image_transport="raw"
        )

        image_transport.subscribe('camera_feed', 10, self.image_callback)
    
    def image_callback(self, msg):
        self.get_logger().info('got a new image from frame_id:=%s' % msg.header.frame_id)

def main(args=None):
    rclpy.init(args=args)
    py_img_sub = PyImgSub()
    rclpy.spin(py_img_sub)

    py_img_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
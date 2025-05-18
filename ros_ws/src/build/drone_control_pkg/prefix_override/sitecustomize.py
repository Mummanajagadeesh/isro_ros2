import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/niraamay/isro_ros2_try/ros_ws/src/install/drone_control_pkg'

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rodrigo/turtlebot3_ws/install/turtlebot3_control_ros2'

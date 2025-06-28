import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/skavrx/aizee/wsl_ws/install/ros_tcp_endpoint'

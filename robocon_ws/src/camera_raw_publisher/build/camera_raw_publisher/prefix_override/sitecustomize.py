import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vboxuser/robocon_ws/src/camera_raw_publisher/install/camera_raw_publisher'

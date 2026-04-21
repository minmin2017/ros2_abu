import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/minmin/robocon_ws/src/robocon_tutorial/install/robocon_tutorial'

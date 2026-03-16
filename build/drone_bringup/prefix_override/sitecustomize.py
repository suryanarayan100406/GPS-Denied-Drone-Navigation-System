import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/LENOVO/OneDrive/Desktop/drone/install/drone_bringup'

import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wilsonchandra/agri_robot_ws/install/agri_robot_description'

import sys
if sys.prefix == '/Users/zdrrrm/.espressif/python_env/idf5.5_py3.10_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/install/ament_lint'

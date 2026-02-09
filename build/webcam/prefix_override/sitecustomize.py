import sys
if sys.prefix == '/opt/homebrew/Caskroom/miniconda/base/envs/ros_env':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/adarshraju/Desktop/Projects/hand_robot_control/install/webcam'

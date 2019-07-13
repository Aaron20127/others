## read HIKVISION
cd catkin_ws
roscore

source devel/setup.bash
rosrun demo sendImage topic 192.168.1.100

source devel/setup.bash
rosrun demo sendImage topic 192.168.1.100

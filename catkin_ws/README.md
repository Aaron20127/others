## read HIKVISION camera, send ros message and read ros message
### start host process
\>\> cd catkin_ws<br>
\>\> roscore<br>

### send ros messag
\>\> source devel/setup.bash<br>
\>\> rosrun demo sendImage topic 192.168.1.100<br>

### read ros message
\>\> source devel/setup.bash<br>
\>\> rosrun demo showImage topic <br>

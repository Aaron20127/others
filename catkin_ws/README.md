## read HIKVISION camera, send ros message and read ros message

### [install ros](http://wiki.ros.org/melodic/Installation/Ubuntu)

```bash
### start host process
\>\> cd catkin_ws<br>
\>\> catkin_cmake<br>
\>\> roscore<br>

### send ros messag
\>\> source devel/setup.bash<br>
\>\> rosrun demo sendImage topic 192.168.1.100<br>

### read ros message
\>\> source devel/setup.bash<br>
\>\> rosrun demo showImage topic <br>
```

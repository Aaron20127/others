## read HIKVISION camera, send ros message and read ros message

### [install ros](http://wiki.ros.org/melodic/Installation/Ubuntu)

```bash
### start host process
>> cd catkin_ws
>> catkin_cmake
>> roscore

### send ros messag
>> source devel/setup.bash
>> rosrun demo sendImage topic 192.168.1.100

### read ros message
>> source devel/setup.bash
>> rosrun demo showImage topic
```

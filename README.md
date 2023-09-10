### Environment: 
OS: Linux Ubuntu 22.04

GAZEBO： Garden

#### command
in one terminal（工作目录切换到有xmate.sdf的目录）
```
. controller/install/setup.bash 
ros2 launch ros2_controller demo.launch.py
```

in another terminal
```
ros2 topic pub --once /rotor1 std_msgs/msg/Float64 "{data: 1.0}"
```

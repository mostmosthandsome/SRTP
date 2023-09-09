### Environment: 
OS: Linux Ubuntu 22.04
GAZEBO： Garden

#### command
in one terminal
```
gz sim xmate.sdf
```

in another terminal
```
gz topic -t "/rotor2" -m gz.msgs.Double -p "data: 0.7"
```

# ROS2 Tutorial

## Variables 

- Set domain name
```bash=1
export ROS_DOMAIN_ID=<domain_id>
```
- Set localhost only
```bash=1
export ROS_LOCALHOST_ONLY=1
```

## CLI tools

### ROS2 pkg

```bash=1
usage: ros2 pkg [-h] Call `ros2 pkg <command> -h` for more detailed usage. ...

Various package related sub-commands

options:
  -h, --help            show this help message and exit

Commands:
  create       Create a new ROS 2 package
  executables  Output a list of package specific executables
  list         Output a list of available packages
  prefix       Output the prefix path of a package
  xml          Output the XML of the package manifest or a specific tag
```

- Check executable package
```bash=1
ros2 pkg executables turtlesim
```

### ROS2 run

```bash=1
ros2 run <package name> <node name>
```

### ROS2 topic

- Show topic list
```bash=1
ros2 topic list
```
- Show topic list with topic type
```bash=1
ros2 topic list -t
```

- Publish
```bash=1
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist \ 
               "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \ 
               "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

- Simple supervise
```bash=1
ros2 topic hz <topic name>
```

### ROS2 interface

- Same as ```rosmsg``` in ROS1

```bash=1
usage: ros2 interface [-h] Call `ros2 interface <command> -h` for more detailed usage. ...

Show information about ROS interfaces

options:
  -h, --help            show this help message and exit

Commands:
  list      List all interface types available
  package   Output a list of available interface types within one package
  packages  Output a list of packages that provide interfaces
  proto     Output an interface prototype
  show      Output the interface definition
```

- Show message
```bash=1
# In ROS1
rosmsg show <message type>
ros2 interface show <message type>
ros2 interface show geometry_msgs/msg/Twist
```
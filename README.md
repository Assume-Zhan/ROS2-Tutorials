# ROS2 Tutorial

## Variables 

- Set domain name
```bash
export ROS_DOMAIN_ID=<domain_id>
```
- Set localhost only
```bash
export ROS_LOCALHOST_ONLY
```

## CLI tools

### ROS2 pkg

```bash
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
```bash
ros2 pkg executables turtlesim
```

### ROS2 run

```bash
ros2 run <package name> <node name>
```

### ROS2 topic

- Show topic list
```bash
ros2 topic list
```
- Show topic list with topic type
```bash
ros2 topic list -t
```

- Publish
```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist \ 
               "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \ 
               "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

- Simple supervise
```bash
ros2 topic hz <topic name>
```

### ROS2 interface

- Same as ```rosmsg``` in ROS1, but it add more **interface**

```bash
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
```bash
# In ROS1
rosmsg show <message type>

# In ROS2
ros2 interface show <message type>
ros2 interface show geometry_msgs/msg/Twist
```
- Show service type
```bash
ros2 interface show <service type>
```

### ROS2 service

- Same as ```rosservice``` in ROS1

- List all services
```bash
ros2 service list
```

- List service type
```bash
ros2 service list -t
ros2 service type <service name>
```

- Service call
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2, y: 2, theta: 0.2, name: ''}"
```

### ROS2 param

- Same as ```rosparam```

- List all params
```bash
ros2 param list
```

- Get parameter
```bash
ros2 param get <node name> <parameter name>
```

- Set parameter
```bash
ros2 param set <node name> <parameter name> <value>
```

- Show all parameter and its value
  - More useful to *pass in yaml file*
```bash
ros2 param dump <node name>
```

- Load parameters
```bash
ros2 param load <node name> <parameter file>
```

### ROS2 launch

- ```roslaunch```

- Launch launch file ( Python or xml )
```bash
ros2 launch turtlesim multisim.launch.py
```

### ROS2 bag

- ```rosbag```

- Record
```bash
ros2 bag record <topic name>
```

- Specified output file name
```bash
ros2 bag record -o hello <topic name>
```

- Play
```bash
ros2 bag play <bag file name>
```

---

## TODO
- learn action [CLI action](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- learn parameter concept in ROS2 
  - [Parameter C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html)
  - [Parameter Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [ROS2 Doctor](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Getting-Started-With-Ros2doctor.html)
- [ROS2 Plugin](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Pluginlib.html)
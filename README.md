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

## Tutorial 2
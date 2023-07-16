# ROS2 Launch

## Create launch

- Create a folder
```bash
# In ws/src/package
mkdir launch
```

- Create a launch file
```bash
# In launch folder
touch test.launch # vim test.launch
```

## Write launch in Python

- Import
```python
from launch import LaunchDescription
from launch_ros.actions import Node
```

## Maintain
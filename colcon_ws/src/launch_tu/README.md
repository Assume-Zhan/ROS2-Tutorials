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
touch test.launch.py # vim test.launch
```

## Write launch in Python

- Import
```python
from launch import LaunchDescription
from launch_ros.actions import Node
```

- Add launch description
    - Return the required part of the launch file
```python
def generate_launch_description():
   return LaunchDescription([

   ])
```

- Add Node description
    - Package : where is the node
    - Namespace : prefix
    - Executable : Node execute name
    - Name : Node name
    - Remap : remap the topics
```python
Node(
    package='turtlesim',
    namespace='turtlesim1',
    executable='turtlesim_node',
    name='sim',
    remappings=[
      ('/input/pose', '/turtlesim1/turtle1/pose'),
      ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
    ]
),
```

## Maintain
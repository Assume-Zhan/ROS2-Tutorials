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

## Basic Usage ( Python )

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

- Important : Add launch dependencies
```xml
<exec_depend>ros2launch</exec_depend>
```

- Important : Install launch folder ```CMakeList.txt```
```txt
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
```

- Launch
```bash
ros2 launch launch_tu test.launch.py
```

## Advanced
> Variable evaluating in **Execution Time**

### Path for other Launch file

- Prepare
```py
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
```

- Find package path
```py
FindPackageShare('package name')
FindPackageShare('launch_tu')
```

- Joint launch file into package path
```py
PathJoinSubstitution([
    "...",
    "..."
])
PathJoinSubstitution([
    FindPackageShare('launch_tu'),
    "launch", "test.launch.py"
])
```

### Launch or Execute With condition


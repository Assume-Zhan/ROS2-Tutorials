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

- Launch with argument
```bash
ros2 launch launch_tu test.launch.py --show-args
```

## Advanced
> Variable evaluating in **Execution Time**

### Argument Declaration

- Prepare
```py
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
```

- Declare an argument which can **be defined by above launch file**
    - We need ```LaunchConfiguration``` to get argument value in any par of launch file
```py
robot_number = LaunchConfiguration('robot_number')
arg = DeclareLaunchArgument(
    "robot_number",
    default_value = "red"
)
```

### Execute CLI command

- Prepare
```py
from launch.actions import DeclareLaunchArgument, ExecuteProcess
```

- Execute the process
```py
spawn_turtle = ExecuteProcess(
    cmd=[[
        'ros2 service call ',
        turtlesim_ns,
        '/spawn ',
        'turtlesim/srv/Spawn ',
        '"{x: 2, y: 2, theta: 0.2}"'
    ]],
    shell=True
)
```

### Launch or Execute With condition

- Prepare
```py
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
```

- In a ```Node``` or ```Execution```
    - You can also put ```Launch Argument into PythonExpression```
```py
condition=IfCondition(
    PythonExpression([
        '1 == 2'
    ])
),
```

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

### Include other launch file

- Prepare
```py
from launch.actions import IncludeLaunchDescription
```

- Include and Launch
    - Launch argument is **dictionary**
```py
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(['path']),
    launch_arguments={
        'ns': 'turtlesim2',
    }.items()
)
```

### Event handler

- Prepare
```py
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
```

- Basic
    - **Target Execution** + **Callback function**
```py
RegisterEventHandler(
    Event(
        target_action = "...",
        on_<event> = [
            function1,
            function2,
            "..."
        ]
    )
)
```

- OnProcessStart
```py
RegisterEventHandler(
    OnProcessStart(
        target_action=turtle1,
        on_start=[
            LogInfo(msg='Start...'),
            spawn_turtle
        ]
    )
)
```

- OnProcessIO
```py
RegisterEventHandler(
    OnProcessIO(
        target_action=spawn_turtle,
        on_stdout=lambda event: LogInfo(
            msg='Spawn request says "{}"'.format(
                event.text.decode().strip())
        )
    )
),
```

- OnExecutionComplete
- OnProcessExit
- OnShutdown
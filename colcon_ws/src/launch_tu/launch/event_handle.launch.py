from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, FindExecutable
from launch.event_handlers import OnExecutionComplete, OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown

from launch_ros.actions import Node

def generate_launch_description():
    
    
    # Declare launch argument
    ns = LaunchConfiguration('ns')
    
    ns_arg = DeclareLaunchArgument(
        "ns", default_value = 'ns1'
    )
    
    # Nodes
    turtle1 = Node(
        namespace = ns,
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )
    
    turtle2 = Node(
        namespace = ns,
        package='turtlesim',
        executable='turtlesim_node',
        name='sim'
    )
    
    turtle_mimic = Node(
        package='turtlesim',
        executable='mimic',
        name='mimic',
        remappings=[
            ('/input/pose', '/turtlesim1/turtle1/pose'),
            ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        ]
    )
    
    spawn_turtle = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            ' service call ',
            ns,
            '/spawn ',
            'turtlesim/srv/Spawn ',
            '"{x: 2, y: 2, theta: 0.2}"'
        ]],
        shell=True
    )
    
    return LaunchDescription([
        ns_arg,
        
        # Nodes
        turtle1, turtle2, turtle_mimic, 
        
        # Event handler
        RegisterEventHandler(
            OnProcessStart(
                target_action=turtle1,
                on_start=[
                    LogInfo(msg='Start...'),
                    spawn_turtle
                ]
            )
        )
    ])
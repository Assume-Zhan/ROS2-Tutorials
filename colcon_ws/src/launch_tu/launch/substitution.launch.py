from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    
    # Declare launch argument
    ns = LaunchConfiguration('ns')
    
    ns_arg = DeclareLaunchArgument(
        'ns', default_value = 'ns2'
    )
    
    # Declare other launch path
    turtle_path = PathJoinSubstitution([
        FindPackageShare('launch_tu'), 
        'launch', 'test.launch.py'
    ])
    
    # Include launch
    include_turtle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtle_path),
        launch_arguments = {
            'ns' : ns
        }.items()
    )
    
    # Return a launch description
    return LaunchDescription([
        ns_arg,
        include_turtle
    ])
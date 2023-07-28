from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_launch_description():
    
    # Declare argument ( Namespace )
    sim_ns = LaunchConfiguration('sim_ns')
    sim_ns_arg = DeclareLaunchArgument(
        'sim_ns', default_value = 'simulation'
    )
    
    # Nodes
    # rviz = Node(
        
    # )
    
    gazebo = Node(
        
    )
    
    # Launch description
    return LaunchDescription([
        # Arguments
        sim_ns_arg,
        
        # Nodes
        gazebo
    ])
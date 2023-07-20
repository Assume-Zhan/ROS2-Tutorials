from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'local_filter.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_velocity',
            remappings=[
                ('/odometry/filtered', '/global_filter'),
                ('/set_pose', '/initialpose'),
            ],
            parameters = [config]
        )
    ])
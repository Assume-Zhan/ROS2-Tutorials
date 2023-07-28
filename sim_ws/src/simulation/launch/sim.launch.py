from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, FindExecutable


def generate_launch_description():
    
    # Declare argument ( Namespace )
    sim_ns = LaunchConfiguration("sim_ns")
    sim_ns_arg = DeclareLaunchArgument(
        "sim_ns", default_value = "simulation"
    )
    # -------------------------------
    
    # Declare more path
    model_path = PathJoinSubstitution([
        FindPackageShare("turtlebot3_gazebo"), 
        "models"
    ])
    
    rviz_path = PathJoinSubstitution([
        FindPackageShare("simulation"), 
        "rviz", "basic_slam_sim.rviz"
    ])
    
    slam_toolbox_path = PathJoinSubstitution([
        FindPackageShare("slam_toolbox"), 
        "launch", "online_sync_launch.py"
    ])
    # -------------------------------
    
    # Setup Environment variables
    gazebo_env = SetEnvironmentVariable(
        name = "GAZEBO_MODEL_PATH", 
        value = model_path
    )
    # -------------------------------
    
    # Nodes
    rviz = Node(
        package = "rviz2",
        namespace = "",
        executable = "rviz2",
        name = "rviz2",
        arguments = ["-d", [rviz_path]]
    )
    
    gazebo = ExecuteProcess(
        cmd=[[
            FindExecutable(name="gazebo"),
        ]],
        shell=True
    )
    
    scan_tf2 = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        name = "foot_to_scan",
        arguments = ["0", "0", "0", "0", "0", "0", "base_footprint", "base_scan"]
        
    )
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_path)
    )
    # -------------------------------
    
    # Launch description
    return LaunchDescription([
        # Arguments
        sim_ns_arg,
        
        # Env
        gazebo_env,
        
        # Nodes
        gazebo, rviz, scan_tf2
    ])
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
        "launch", "online_async_launch.py"
    ])
    
    gazebo_path = PathJoinSubstitution([
        FindPackageShare("gazebo_ros"), 
        "launch", "gazebo.launch.py"
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
        arguments = ["-d", [rviz_path]],
        parameters=[{'use_sim_time': False}]
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path)
    )
    
    # Static TF : base_footprint -> base_link
    foot_to_link_tf2 = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        name = "foot_to_link",
        arguments = ["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
        parameters=[{'use_sim_time': True}]
    )
    
    # Static TF : base_link -> base_scan
    link_to_scan_tf2 = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        name = "link_to_scan",
        arguments = ["0", "0", "0", "0", "0", "0", "base_link", "base_scan"],
        parameters=[{'use_sim_time': True}]
    )
    
    # Static TF : base_link -> imu
    link_to_imu_tf2 = Node(
        package = "tf2_ros",
        executable = "static_transform_publisher",
        name = "link_to_imu",
        arguments = ["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
        parameters=[{'use_sim_time': True}]
    )
    
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_path),
        launch_arguments={
            'use_sim_time': "True",
        }.items()
    )
    # -------------------------------
    
    # Launch description
    return LaunchDescription([
        # Arguments
        sim_ns_arg,
        
        # Env
        gazebo_env,
        
        # Nodes or Launch
        rviz, gazebo, foot_to_link_tf2, link_to_scan_tf2, link_to_imu_tf2
    ])
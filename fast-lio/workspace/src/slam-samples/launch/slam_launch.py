import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_path = get_package_share_directory("slam-samples")
    config_dir_path = os.path.join(package_path, "config")
    launch_dir_path = os.path.join(package_path, "launch")


    # launch lidar_launch.py
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_dir_path, "/lidar_launch.py"]),
        launch_arguments={
            "xfer_format": "0",
            "lidar_config_path": os.path.join(config_dir_path, "MID360_config.json"),
            "rviz": "false",
        }.items(),
    )

    # launch pointcloud_filter_node
    pointcloud_filter_node = Node(
        package="pointcloud_filter",
        executable="pointcloud_filter_node",
        output="screen",
        parameters=[
        ],
        remappings=[
            ("/in_cloud", "/livox/lidar"),
            ("/out_cloud", "/livox/filtered_lidar")
        ]
    )

    # launch fast_lio node.`
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_dir_path, "fastlio_config.yaml"]),
        ],
        remappings=[
            ("/Odometry", "/fastlio_odom"),
        ]
    )

    static_transform_publisher_node_odom_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_odom_to_livox",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "livox_frame"]
    )

    static_transform_publisher_node_livox_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_livox_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "livox_frame", "base_footprint"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[
        ],
        arguments=[
            "-d",
            os.path.join(
                get_package_share_directory("slam-samples"),
                "rviz",
                "slam.rviz",
            ),
        ],
    )

    return LaunchDescription(
        [
            lidar_launch,
            pointcloud_filter_node,
            fast_lio_node,
            static_transform_publisher_node_odom_to_livox,
            static_transform_publisher_node_livox_to_base,
            rviz_node,
        ]
    )
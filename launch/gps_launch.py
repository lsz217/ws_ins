# launch/gps_launch.py
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Launching GPS Processing Nodes..."),

        ComposableNodeContainer(
            name='gps_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='ins_processing',
                    plugin='ins_processing::GpsPublisher',
                    name='gps_publisher',
                    parameters=[{"file_path": "/home/lsz/下载/gps.txt"}]
                ),
                ComposableNode(
                    package='ins_processing',
                    plugin='ins_processing::GpsConverter',
                    name='gps_converter'
                ),
                ComposableNode(
                    package='ins_processing',
                    plugin='ins_processing::GpsSubscriber',
                    name='gps_subscriber'
                )
            ],
            output='screen',
            emulate_tty=True
        )
    ])










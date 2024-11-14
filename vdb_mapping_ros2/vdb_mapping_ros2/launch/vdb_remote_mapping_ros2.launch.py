import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
            get_package_share_directory('vdb_mapping_ros2'),
            'config',
            'vdb_remote_params.yaml'
            )

    vdb_mapping = Node(
        package='vdb_mapping_ros2',
        executable='vdb_mapping_ros_node',
        name='vdb_remote_mapping',
        parameters = [config]
    )

    ld.add_action(vdb_mapping)

    return ld

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
            get_package_share_directory('vdb_mapping_ros2'),
            'config',
            'vdb_params.yaml'
            )

    vdb_mapping = Node(
        package='vdb_mapping_ros2',
        executable='vdb_mapping_ros_node',
        name='vdb_mapping',
        parameters = [config, {'use_sim_time': True}],
    )

    # Params from
    # https://github.com/robot-pesg/BotanicGarden/blob/main/calib/extrinsics/calib_chain.yaml
    lidar_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne_in_xsense',
        arguments = ['0.0584867781527745', '0.00840419966766332', '0.168915521980526', '0.0078031', '0.0015042', '-0.0252884', 'base_link', 'velodyne'],
        parameters = [{'use_sim_time': True}],
    )

    map_to_base_tf = Node(
        package='vdb_mapping_ros2',
        executable='tf_publisher.py',
        name='map_to_base_link',
        parameters = [{'use_sim_time': True}],
    )


    ld.add_action(vdb_mapping)
    ld.add_action(lidar_to_base_tf)
    ld.add_action(map_to_base_tf)

    return ld

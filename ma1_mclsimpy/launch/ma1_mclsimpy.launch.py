import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ma1_mclsimpy'),
        'config',
        'sim_params.yaml'
    )

    mclsimpy_node = Node(
        package='ma1_mclsimpy',
        executable='sim_with_waves.py',
        name='ma1_mclsimpy_node',
        parameters=[config],
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(mclsimpy_node)

    return ld

import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
nav2_bringup_dir = get_package_share_directory('nav2_bringup')
rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
def generate_launch_description():
    return launch.LaunchDescription([
            launch_ros.actions.Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_dir],
                parameters=[{'use_sim_time': True}],
                output='screen'),
        ])
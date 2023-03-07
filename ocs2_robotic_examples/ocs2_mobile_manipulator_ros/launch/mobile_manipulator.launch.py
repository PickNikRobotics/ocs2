
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ocs2_robotic_assets_dir = get_package_share_directory('ocs2_robotic_assets')
    ocs2_mobile_manipulator_dir = get_package_share_directory('ocs2_mobile_manipulator')

    task_file = os.path.join(ocs2_mobile_manipulator_dir, 'config', 'mabi_mobile', 'task.info')
    urdf_file = os.path.join(ocs2_robotic_assets_dir, 'resources', 'mobile_manipulator', 'mabi_mobile', 'urdf', 'mabi_mobile.urdf')
    lib_folder = os.path.join(ocs2_robotic_assets_dir, 'auto_generated', 'kinova', 'mabi_mobile')
    # task_file = os.path.join(ocs2_mobile_manipulator_dir, 'config', 'franka', 'task.info')
    # urdf_file = os.path.join(ocs2_robotic_assets_dir, 'resources', 'mobile_manipulator', 'franka', 'urdf', 'panda.urdf')
    # lib_folder = os.path.join(ocs2_robotic_assets_dir, 'auto_generated', 'franka', 'panda')
    use_sim_time = False

    urdf_content = ""
    with open(urdf_file, 'r') as fp:
        urdf_content = fp.read()

    robot_description = {"robot_description": urdf_content}

    return LaunchDescription([

        Node(
            name='mobile_manipulator_mpc_node',
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            parameters=[
                {"use_sim_time": use_sim_time},
                {"urdf_file": urdf_file},
                {"task_file": task_file},
                {"lib_folder": lib_folder},
                ],
            output='screen',
            # prefix=["xterm -e"],
        ),
        Node(
            name='mobile_manipulator_dummy_mrt_node',
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_dummy_mrt_node',
            parameters=[
                robot_description,
                {"use_sim_time": use_sim_time},
                {"urdf_file": urdf_file},
                {"task_file": task_file},
                {"lib_folder": lib_folder},
                ],
            output='screen',
            prefix=["xterm -e"],
        ),
        Node(
            name='mobile_manipulator_target',
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_target',
            parameters=[
                {"use_sim_time": use_sim_time},
                ],
            output='screen',
            # prefix=["xterm -e"],
        ),
        # Node(
        #     name='mobile_manipulator_distance_visualization',
        #     package='ocs2_mobile_manipulator_ros',
        #     executable='mobile_manipulator_distance_visualization',
        #     parameters=[
        #         {"use_sim_time": use_sim_time},
        #         {"urdf_file": urdf_file},
        #         {"task_file": task_file},
        #         ],
        #     output='screen',
        #     # prefix=["xterm -e"],
        # ),
    ])


def main(argv=sys.argv[1:]):
    """Run lifecycle nodes via launch."""
    ld = generate_launch_description()
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Cesta ke konfiguraci rviz a launch souboru simulátoru
    package_dir = get_package_share_directory('mpc_rbt_student')
    rviz_config_path = os.path.join(package_dir, 'rviz', 'my.rviz')
    simulator_launch = os.path.join(get_package_share_directory('mpc_rbt_simulator'), 'launch', 'simulation.launch.py')
    
    # Vytvoření LaunchDescription
    return LaunchDescription([

        # Spuštění rviz2 s konfigurací
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],  # Cesta k tvé konfiguraci RViz
            output='screen'
        ),

        # TimerAction pro spuštění simulátoru po 3 sekundách
        TimerAction(
            period=3.0,  # Po 3 sekundách
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(simulator_launch)  # Spuštění simulátoru
                )
            ]
        ),
        
        # TimerAction pro spuštění localization_node a keyboard_control po 10 sekundách
        TimerAction(
            period=10.0,  # Po 10 sekundách
            actions=[
                Node(
                    package='mpc_rbt_student',
                    executable='localization_node',
                    name='localization_node',
                    output='screen'
                ),
                
                Node(
                    package='mpc_rbt_student',
                    executable='planning_node',
                    name='planning_node',
                    output='screen'
                ),

                Node(
                    package='mpc_rbt_student',
                    executable='motion_control_node',
                    name='motion_control_node',
                    output='screen'
                ),
                ExecuteProcess(
                    cmd=['gnome-terminal', '--', 'ros2', 'run', 'mpc_rbt_student', 'motion_control_node'],
                    name='motion_control_terminal',
                    output='screen'
                )
                
            ]
        )
    ])

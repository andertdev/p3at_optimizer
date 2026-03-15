from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context).strip().lower()
    robot_ip = LaunchConfiguration('robot_ip').perform(context).strip()
    robot_port = int(LaunchConfiguration('robot_port').perform(context))
    parser_mode = LaunchConfiguration('parser_mode').perform(context).strip().lower()
    dual_route = LaunchConfiguration('dual_route').perform(context).strip().lower()

    pkg_control = get_package_share_directory('p3at_control')
    params_file = os.path.join(pkg_control, 'config', 'params.yaml')

    nodes = []

    # =========================================================
    # SIM (Webots manual + controller UDP)
    # - Webots você abre manual e dá Play
    # - controller p3at_webots_controller recebe UDP
    # - ROS só sobe: api(mode=sim) + bus_udp
    # =========================================================
    if mode == 'sim':
        nodes.append(
            Node(
                package='p3at_control',
                executable='bus_udp',
                name='p3at_bus_udp',
                output='screen',
                parameters=[params_file]
            )
        )

        nodes.append(
            Node(
                package='p3at_control',
                executable='api',
                name='api',
                output='screen',
                parameters=[params_file, {
                    'mode': 'sim',
                    'use_executor': True,
                    'parser_mode': parser_mode,
                    'dual_route': dual_route
                }]
            )
        )
        return nodes

    # =========================================================
    # REAL (pipeline ROS completo)
    # =========================================================
    if mode == 'real':
        # DRIVER REAL
        nodes.append(
            Node(
                package='simple_p3at_driver',
                executable='simple_p3at_node',
                name='p3at_real_driver',
                output='screen',
                parameters=[params_file, {'robot_ip': robot_ip, 'robot_port': robot_port}]
            )
        )

        # EXECUTOR (traduz comandos JSON -> cmd_vel direto, sem optimizer)
        nodes.append(
            Node(
                package='p3at_control',
                executable='executor',
                name='executor',
                output='screen',
                parameters=[params_file, {'direct_cmd_vel': True}]
            )
        )

        # API
        nodes.append(
            Node(
                package='p3at_control',
                executable='api',
                name='api',
                output='screen',
                parameters=[params_file, {
                    'mode': 'real',
                    'use_executor': True,
                    'parser_mode': parser_mode,
                    'dual_route': dual_route
                }]
            )
        )

        return nodes

    raise RuntimeError(f"Modo inválido: {mode} (use sim ou real)")


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='sim',
            description='Modo de execução: sim ou real'
        ),
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.0.1',
            description='IP do robô real'
        ),
        DeclareLaunchArgument(
            'robot_port',
            default_value='20001',
            description='Porta do robô real'
        ),
        DeclareLaunchArgument(
            'parser_mode',
            default_value='llm',
            description='Parser mode: llm | cap | dual'
        ),
        DeclareLaunchArgument(
            'dual_route',
            default_value='llm_fallback_cap',
            description='Dual route: llm_fallback_cap | cap_fallback_llm'
        ),
        OpaqueFunction(function=launch_setup)
    ])

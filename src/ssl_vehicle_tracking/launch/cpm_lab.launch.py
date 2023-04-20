from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch import LaunchDescription, LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs) -> list[Node]:
    use_sim_time: LaunchConfiguration = LaunchConfiguration(
        'use_sim_time', default='false')
    sensor_id: LaunchConfiguration = LaunchConfiguration('sensor_id')
    ssl_ids: LaunchConfiguration = LaunchConfiguration('ssl_ids')

    namespace_str: str = '/ssl_vehicle_detection_' + \
        context.perform_substitution(sensor_id)
    ssl_id_list_str: str = context.perform_substitution(ssl_ids)
    ssl_id_list_int: list[int] = []
    for ssl_id in ssl_id_list_str.split(','):
        ssl_id_list_int.append(int(ssl_id))

    nodes: list[Node] = []

    for ssl_id in ssl_id_list_int:
        wheel_detection: Node = Node(
            package='ssl_vehicle_tracking_wheel_detection',
            executable='ssl_vehicle_tracking_wheel_detection_node',
            name='wheel_detection_' + str(ssl_id),
            output='screen',
            namespace=namespace_str,
            parameters=[{'use_sim_time': use_sim_time, 'ssl_id': ssl_id}],
            remappings=[
                    (namespace_str + '/ssl_' + str(ssl_id), '/ssl_' + str(ssl_id)),
            ]
        )
        nodes.append(wheel_detection)

    wheel_tracking: Node = Node(
        package='ssl_vehicle_tracking_wheel_tracking',
        executable='ssl_vehicle_tracking_wheel_tracking_node',
        name='wheel_tracking',
        output='screen',
        namespace=namespace_str,
        parameters=[{'use_sim_time': use_sim_time, 'ssl_ids': ssl_id_list_int}]
    )
    nodes.append(wheel_tracking)
    vehicle_detection: Node = Node(
        package='ssl_vehicle_tracking_vehicle_detection',
        executable='ssl_vehicle_tracking_vehicle_detection_node',
        name='vehicle_detection',
        output='screen',
        namespace=namespace_str,
        parameters=[{'use_sim_time': use_sim_time}]
    )
    nodes.append(vehicle_detection)
    return nodes


def generate_launch_description() -> LaunchDescription:

    launch = LaunchDescription([
        DeclareLaunchArgument(
            'sensor_id',
            description='Id of this infrastructure sensor.',
            choices=['1', '2', '3', '4']
        ),
        DeclareLaunchArgument(
            'ssl_ids',
            description='Ids of ssl layers that are input to this tracking application. Use comma seperated list without spaces.',
            default_value="1,2"
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            description='Flag indicating that a recorded file is used.',
            choices=['false', 'true']
        ),
        LogInfo(msg=['Infrastructure sensor driver package started with id ',
                LaunchConfiguration('sensor_id'), '.']),
        LogInfo(msg=['Infrastructure sensor source selected as ssl ids ',
                LaunchConfiguration('ssl_ids'), '.']),
        LogInfo(msg=[_clock_message('use_sim_time')]),
        # PushRosNamespace(['infrastructure_', LaunchConfiguration('sensor_id')]),
        OpaqueFunction(function=launch_setup),
    ])

    return launch


def _clock_message(arg) -> PythonExpression:
    cmd = ['"Vehicle driver package " + "uses simualted clock." if "true" == "',
           LaunchConfiguration(arg), '" else "uses real-time clock."']
    py_cmd = PythonExpression(cmd)
    return py_cmd

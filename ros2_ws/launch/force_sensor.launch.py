from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    interface_arg = DeclareLaunchArgument(
        'interface',
        default_value='enp8s0',
        description='EtherCAT network interface name'
    )
    
    cycle_time_us_arg = DeclareLaunchArgument(
        'cycle_time_us',
        default_value='8000',
        description='Sensor data cycle time in microseconds (8000us = 125Hz)'
    )
    
    # 创建节点
    force_sensor_node = Node(
        package='ros2_force_sensor',
        executable='force_sensor_node',
        name='force_sensor_node',
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'cycle_time_us': LaunchConfiguration('cycle_time_us'),
            'fx_offset': -16.35,
            'fy_offset': -6.70,
            'fz_offset': 12.35,
        }],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        interface_arg,
        cycle_time_us_arg,
        force_sensor_node
    ])
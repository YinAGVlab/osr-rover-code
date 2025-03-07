import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 
 
def generate_launch_description():

    ld = LaunchDescription()
    
    # 参数配置文件
    roboclaw_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'roboclaw_params.yaml'
    )
    osr_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'osr_params.yaml'
    )

    # 可选参数
    ld.add_action(DeclareLaunchArgument('enable_odometry', default_value='false'))
    ld.add_action(DeclareLaunchArgument('joy_vel', default_value='cmd_vel'))

    # 驱动板节点
    ld.add_action(
        Node(
            package='osr_control',
            executable='roboclaw_wrapper',
            name='roboclaw_wrapper',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[roboclaw_params]
        )
    )

    # 舵机节点
    ld.add_action(
        Node(
            package='osr_control',
            executable='servo_control',
            name='servo_wrapper',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[{'centered_pulse_widths': [165, 223, 153, 222]}]  # pulse width where the corner motors are in their default position, see rover_bringup.md.
        )
    )
    
    # 整体控制
    ld.add_action(
        Node(
            package='osr_control',
            executable='rover',
            name='rover',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[osr_params,
                        {'enable_odometry': LaunchConfiguration('enable_odometry')}]
        )
    )

    # 手柄遥控信号转控制信号节点
    ld.add_action(
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            # output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[
                {"axis_linear.x": 1},  # 前进后退摇杆
                {"scale_linear.x": 1.0},  # 行进速度  0.6
                # {"scale_linear_turbo.x": 1.78},  # 速度单位，Xm/s?
                {"axis_angular.yaw": 3},  # 转向摇杆
                {"scale_angular.yaw": -1.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                # {"scale_angular_turbo.yaw": 3.95},  # scale to apply to angular speed, in rad/s: scale_linear_turbo / min_radius

                {"axis_angular.pitch": 6},  # 原地转向摇杆
                {"scale_angular.pitch": 0.5},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)

                # {"enable_button": 4},  # which button to press to enable movement
                {"enable_button": 0},  # which button to press to enable movement
                # {"enable_turbo_button": 5}  # -1 to disable turbo
                {"enable_turbo_button": -1},  # -1 to disable turbo
            ],
            remappings=[
                ('/cmd_vel', LaunchConfiguration('joy_vel'))
            ]
        )
    )

    # 手柄节点
    ld.add_action(
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[
                {"autorepeat_rate": 5.0},
                {"device_id": 0},  # This might be different on your computer. Run `ls -l /dev/input/event*`. If you have event1, put 1.
            ]        
        )
    )

    
    
    return ld

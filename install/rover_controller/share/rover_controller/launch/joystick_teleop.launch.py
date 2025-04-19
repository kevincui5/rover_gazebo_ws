import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    rover_controller_pkg = get_package_share_directory('rover_controller')

    use_sim_time_arg = DeclareLaunchArgument(name="use_sim_time", default_value="True",
                                      description="Use simulated time"
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[os.path.join(get_package_share_directory("rover_controller"), "config", "joy_teleop.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[os.path.join(get_package_share_directory("rover_controller"), "config", "joy_config.yaml"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )
    
    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "rover_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(rover_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(rover_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(rover_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    twist_relay_node = Node(
        package="rover_controller",
        executable="twist_relay",
        name="twist_relay",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]
    )

    joy_teleop2 = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            emulate_tty=True,
            respawn=True,
            parameters=[
                # {"scale_linear.x": 0.4},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
                {"scale_linear.x": 0.40},  # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
                # {"axis_linear.x": 4},
                {"axis_linear.x": 1},
                # {"axis_angular.yaw": 0},  # which joystick axis to use for driving
                {"axis_angular.yaw": 3},  # which joystick axis to use for driving
                # {"scale_angular.yaw": 1.25},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"axis_angular.pitch": 4},  # axis to use for in-place rotation
                {"scale_angular.yaw": 0.05},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"scale_angular.pitch": 0.0},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
                {"scale_angular_turbo.yaw": 0.55},  # scale to apply to angular speed, in rad/s: scale_linear_turbo / min_radius
                {"scale_linear_turbo.x": 0.90},  # scale to apply to linear speed, in m/s
                # {"enable_button": 4},  # which button to press to enable movement
                {"enable_button": 5},  # which button to press to enable movement
                # {"enable_turbo_button": 5},  # -1 to disable turbo
                {"enable_turbo_button": 3},  # -1 to disable turbo
                {"publish_stamped_twist": True},  # publish a stamped twist message
            ],
            remappings=[
                ('/cmd_vel', '/input_joy/cmd_vel_stamped')
            ]
        )
    
    joy_node2 = Node(
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
    
    
    return LaunchDescription(
        [
            use_sim_time_arg,
            joy_teleop,
            joy_node,
            twist_mux_launch,
            twist_relay_node,
        ]
    )

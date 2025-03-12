from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "type",
            default_value="husky_basic",
            choices=["husky_basic", "husky_manipulator", "husky_ur"],
            description="Specify the robot model to be launched (e.g., 'husky_basic', 'husky_manipulator', 'husky_ur')",
        ),
    ]

    robot_yaml_path = LaunchConfiguration("type")

    setup_path = PathJoinSubstitution(["/dev_ws/src/husky_commander/config", robot_yaml_path])

    husky_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            FindPackageShare("clearpath_gz").find("clearpath_gz") + "/launch/simulation.launch.py"
        ),
        launch_arguments={"setup_path": setup_path}.items(),
    )

    return LaunchDescription(declared_arguments + [husky_simulation_launch])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
  use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

  def config(package, name):
      return PathJoinSubstitution([FindPackageShare(package), "config", LaunchConfiguration(name)])

  return LaunchDescription([
      DeclareLaunchArgument("use_sim_time", default_value="true",
                            description="Follow /clock (bag played with --clock)."),
      DeclareLaunchArgument("scan_to_constraint_config", default_value="scan_to_constraint.yaml"),
      DeclareLaunchArgument("cbf_config", default_value="cbf.yaml"),

      Node(package="pcd_constraints", executable="scan_to_constraint_app", output="screen",
            parameters=[config("pcd_constraints", "scan_to_constraint_config"), {"use_sim_time": use_sim_time}]),
      Node(package="cbf_safety_filter", executable="cbf_app", output="screen",
            parameters=[config("cbf_safety_filter", "cbf_config"), {"use_sim_time": use_sim_time}]),
  ])
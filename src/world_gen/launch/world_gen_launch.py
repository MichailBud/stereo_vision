import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Полный путь к файлу модели мира
    world_path = '/home/michail/BIIS/stereo_vision/worlds/my_world2'

    #Полный путь к файлу модели робота
    robot_path = '/home/michail/BIIS/stereo_vision/models/robot_cameron/model.sdf'
    robot_xml = open(robot_path, 'r').read()
    robot_xml = robot_xml.replace('"', '\\"')
    robot_spawn_args = '{name: \"robot_vac\", xml: \"' + robot_xml + '\" }'


    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', robot_spawn_args],
            output='screen')

])

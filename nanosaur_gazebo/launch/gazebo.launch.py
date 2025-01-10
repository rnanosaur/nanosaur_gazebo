# Copyright (C) 2024, Raffaello Bonghi <raffaello@rnext.it>
# All rights reserved
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
# EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

class Coordinate:
    
    def safe_list_get(self, l, idx, default=0.0):
        try:
            return str(l[idx])
        except IndexError:
            return str(default)

    def __init__(self, config={}) -> None:
        position = config.get('xyz', [])
        orientation = config.get('RPY', [])
        self.x = self.safe_list_get(position, 0)
        self.y = self.safe_list_get(position, 1)
        self.z = self.safe_list_get(position, 2)
        self.R = self.safe_list_get(orientation, 0)
        self.P = self.safe_list_get(orientation, 1)
        self.Y = self.safe_list_get(orientation, 2)
        
    def __repr__(self) -> str:
        coordinate = f"xyz=[{self.x} {self.y} {self.z}] RPY=[{self.R} {self.P} {self.Y}]"
        return coordinate


def load_robot_position(config, world_file_name):
    # Extract worldfile name from configuration
    world_name = Path(world_file_name).stem
    # Check fi file exist
    if not os.path.isfile(config):
        print("no file available")
        return Coordinate()
    # Load yml file
    with open(config, "r") as stream:
        try:
            robot_config = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return Coordinate()
    # Check if world exist
    if world_name not in robot_config:
        return Coordinate()
    # load position and orientation
    config = robot_config[world_name]
    # Extract configuration 
    return Coordinate(config)


def launch_gazebo_setup(context: LaunchContext, support_namespace, support_world):
    """ Reference:
        https://answers.ros.org/question/396345/ros2-launch-file-how-to-convert-launchargument-to-string/ 
        https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_moveit_config/launch/ur_moveit.launch.py
    """
    package_gazebo = get_package_share_directory('nanosaur_gazebo')
    package_worlds = get_package_share_directory('nanosaur_worlds')
    # render namespace, dumping the support_package.
    namespace = context.perform_substitution(support_namespace)
    world_name = context.perform_substitution(support_world) + '.sdf'
    print(f"Loading world: {world_name}")
    gui_config = os.path.join(package_gazebo, "gui", "gui.config")
    basic_world = os.path.join(package_worlds, "worlds", world_name)
    
    # Load configuration from params
    conf = load_robot_position(os.path.join(package_worlds, 'params', 'spawn_robot.yml'), world_name)
    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        namespace=namespace,
        arguments=['-topic', 'robot_description',
                   '-name', 'nanosaur',
                   '-allow_renaming', 'true',
                   '-x', conf.x, '-y', conf.y, '-z',conf.z,
                   '-R', conf.R, '-P', conf.P, '-Y',conf.Y,
                   ],
    )
    
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ros_gz_sim'),
                                                    'launch', 'gz_sim.launch.py')]),
        launch_arguments=[('gz_args', [' -r -v 3 ' + basic_world + ' '
                                        + ' --gui-config ' + gui_config
                                        ])])
    
    return [gazebo_spawn_entity, ign_gazebo]
    

def generate_launch_description():
    package_gazebo = get_package_share_directory('nanosaur_gazebo')

    default_world_name = 'lab' # Empty world: empty

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace = LaunchConfiguration('namespace', default="nanosaur")
    world_name = LaunchConfiguration('world_name', default=default_world_name)
    
    launch_file_dir = os.path.join(package_gazebo, 'launch')

    # Set gazebo resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH', value=[
            os.path.join(get_package_prefix('nanosaur_description'), "share"),
            ":" +
            os.path.join(get_package_share_directory('nanosaur_worlds'), "models")])

    use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    nanosaur_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='nanosaur',
        description='nanosaur namespace name. If you are working with multiple robot you can change this namespace.')

    world_name_cmd = DeclareLaunchArgument(
        name='world_name',
        default_value=default_world_name,
        description='Simulation world name.')

    rsp_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/robot_state_publisher.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    ros_gz_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [launch_file_dir, '/ros_gz_bridge.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time, 'world_name': world_name}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(ign_resource_path)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(nanosaur_cmd)
    ld.add_action(world_name_cmd)
    ld.add_action(OpaqueFunction(function=launch_gazebo_setup, args=[namespace, world_name]))
    ld.add_action(rsp_launcher)
    ld.add_action(ros_gz_bridge)

    return ld
# EOF

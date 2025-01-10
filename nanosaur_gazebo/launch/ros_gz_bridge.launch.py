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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals


def launch_gz_bridge_setup(context: LaunchContext, support_use_sim_time, support_namespace, support_world_name):

    use_sim_time = context.perform_substitution(support_use_sim_time)
    # Cast in boolean
    use_sim_time = use_sim_time.lower() in ("true", "1", "yes", "on")
    namespace = context.perform_substitution(support_namespace)
    world_name = context.perform_substitution(support_world_name)
    
    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # cmd_vel bridge
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/Imu@ignition.msgs.IMU'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # cmd_vel bridge
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_bridge',
        output='screen',
        namespace=namespace,
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[f'/model/{namespace}/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                    f'/world/{world_name}/model/{namespace}/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                    ],
        remappings=[
            (f'/model/{namespace}/cmd_vel', 'cmd_vel'),
            (f'/world/{world_name}/model/{namespace}/joint_state', 'joint_states'),
        ]
        )

    ###################### Camera ######################
    
    # realsense infra1 bridge
    realsense_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='realsense_bridge',
        output='screen',
        namespace='camera',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['/infra1/camera_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                    '/infra1/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                    '/infra2/camera_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                    '/infra2/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
        remappings=[
            ('/infra1/camera_raw', 'infra1/image_raw'),
            ('/infra1/camera_info', 'infra1/camera_info'),
            ('/infra2/camera_raw', 'infra2/image_raw'),
            ('/infra2/camera_info', 'infra2/camera_info')
        ],
        condition=LaunchConfigurationEquals('head_type', 'realsense')
    )

    # include another launch file in nanosaur namespace
    camera_group = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(namespace),
            # nanosaur cameras
            realsense_bridge
        ]
    )
    return [cmd_vel_bridge, camera_group]


def generate_launch_description():
    pkg_control = get_package_share_directory('nanosaur_control')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world_name')
    ##############################àworld_name = "lab"
    head_type = LaunchConfiguration('head_type')
    flap_type = LaunchConfiguration('flap_type')
    namespace = LaunchConfiguration('namespace')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    world_name_cmd = DeclareLaunchArgument(
        name='world_name',
        default_value='empty',
        description='Name of the world you are loading')

    nanosaur_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='nanosaur',
        description='nanosaur namespace name. If you are working with multiple robot you can change this namespace.')

    declare_head_type_cmd = DeclareLaunchArgument(
        name='head_type',
        default_value='realsense',
        description='Head type to use. Options: empty, Realsense, zed.')

    declare_flap_type_cmd = DeclareLaunchArgument(
        name='flap_type',
        default_value='empty',
        description='Flap type to use. Options: empty, LD06.')

    ###################### Twist controls ######################

    # include another launch file in nanosaur namespace
    twist_control_launch = GroupAction(
        actions=[
            # push-ros-namespace to set namespace of included nodes
            PushRosNamespace(namespace),
            # nanosaur twist launch
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([pkg_control, '/launch/twist_control.launch.py']))
        ]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_cmd)
    ld.add_action(world_name_cmd)
    ld.add_action(declare_head_type_cmd)
    ld.add_action(declare_flap_type_cmd)
    ld.add_action(nanosaur_cmd)
    ld.add_action(OpaqueFunction(function=launch_gz_bridge_setup, args=[use_sim_time, namespace, world_name]))
    ld.add_action(twist_control_launch)

    return ld
# EOF

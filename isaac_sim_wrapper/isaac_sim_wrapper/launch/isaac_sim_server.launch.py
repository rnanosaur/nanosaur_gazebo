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
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction


def launch_setup(context: LaunchContext, support_isaac_sim_folder):
    package_isaac_sim = get_package_share_directory('isaac_sim_wrapper')
    # render namespace, dumping the support_package.
    isaac_sim_folder = context.perform_substitution(support_isaac_sim_folder)
    isaac_sim_version = isaac_sim_folder.split('-')[-1]
    # Get the user's home directory
    user_home_dir = os.path.expanduser("~")
    isaac_sim_path = f"{user_home_dir}/.local/share/ov/pkg/{isaac_sim_folder}"
    # Config path
    package_isaac_sim_config = os.path.join(package_isaac_sim, "config", 'isaac_sim.yaml')

    print(f"Run Isaac Sim {isaac_sim_version} from {isaac_sim_path}")
    # Path Launcher Isaac Sim
    isaac_sim_wrapper_launcher = os.path.join(package_isaac_sim, "scripts", "isaac_sim_robot_launcher.py")
    # Start Isaac Sim from python script
    isaac_sim = ExecuteProcess(
            cmd=[f"{isaac_sim_path}/python.sh", isaac_sim_wrapper_launcher, package_isaac_sim_config],
            name='IsaacSim',
            output='screen',
            shell=True
        )
    
    return [isaac_sim]


def generate_launch_description():

    isaac_sim_folder_cmd = DeclareLaunchArgument(
        name='isaac_sim_folder',
        default_value='prod-isaac-sim-4.1.0',
        description='Folder name Isaac Sim version')
    
    isaac_sim_folder = LaunchConfiguration('isaac_sim_folder')

    ld = LaunchDescription()
    ld.add_action(isaac_sim_folder_cmd)
    ld.add_action(OpaqueFunction(function=launch_setup, args=[isaac_sim_folder]))
    
    return ld
# EOF
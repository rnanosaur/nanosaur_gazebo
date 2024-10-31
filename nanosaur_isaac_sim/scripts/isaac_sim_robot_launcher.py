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

import yaml
import argparse
from isaacsim import SimulationApp
# https://docs.omniverse.nvidia.com/isaacsim/latest/installation/install_python.html


def initialize_simulation_app(
                        renderer: str = "RayTracedLighting",
                        headless: bool = False,
                        file_path: str ="config.yaml"):
    """Initialize the Simulation Application."""
    # Build simulation config
    default_config = {"renderer": renderer, "headless": headless}
    # load file
    try:
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)
    except FileNotFoundError:
        print(f"Configuration file {file_path} not found. Using default settings.")
        config = {}
    # Use default values if some settings are not defined
    simulation_config = {**default_config, **config}
    simulation_app = SimulationApp(simulation_config)
    # Load extensions
    from omni.isaac.core.utils.extensions import enable_extension
    # enable ROS2 bridge extension
    enable_extension("omni.isaac.ros2_bridge")
    simulation_app.update()
    return simulation_app


def main():
    # Create the argument parser
    parser = argparse.ArgumentParser(
        description="Isaac Sim Wrapper. Load Isaac Sim and bridge with URDF"
    )
    parser.add_argument("file_path", type=str, help="Isaac Sim file path configuration")

    # Parse the arguments
    args = parser.parse_args()
    # Load Isaac Sim with ROS 2 extension enabled
    simulation_app = initialize_simulation_app(args.file_path)
    # Load the Isaac World library
    import isaac_world
    # Start ros 2 Isaac World controller
    try:
        isaac_world.ros_bridge_main(simulation_app)
    except isaac_world.IsaacWorldError as e:
        print(f"Error: {e.message}. Got {e.value}")
    # Shutdown simulation
    # simulation_app.close()


if __name__ == "__main__":
    main()
# EOF
